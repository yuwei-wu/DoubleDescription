// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <chrono>


namespace opt_planner
{

  // SECTION interfaces for setup and query

  PlannerManager::PlannerManager() {}

  PlannerManager::~PlannerManager() { std::cout << "des manager" << std::endl; }

  void PlannerManager::initPlanModules(ros::NodeHandle &nh, 
                                       const std::shared_ptr<MPL::VoxelMapUtil> &map_util)
  {
    /* read algorithm parameters */

    /* general parameters */
    nh.param("max_v",  pp_.max_vel_, -1.0);
    nh.param("max_a",  pp_.max_acc_, -1.0);
    nh.param("max_j",  pp_.max_jerk_, -1.0);

    nh.param("map_frame", frame_id_, std::string(""));

    /* optimization parameters */
    nh.param("optimization/bb_back", bb_back_, 0.5);
    Eigen::VectorXd w_total(4), b_total(2);
    nh.param("optimization/w_time", w_total(0), 32.0);
    nh.param("optimization/w_vel", w_total(1), 128.0);
    nh.param("optimization/w_acc", w_total(2), 128.0);
    nh.param("optimization/w_sta_obs", w_total(3), 128.0);
    nh.param("search/use_jerk", use_jerk_, false);
    nh.param("search/time_res", time_res_, 0.1);


    b_total << pp_.max_vel_, pp_.max_acc_;

    local_data_.traj_id_ = 0;
    /*  map intial  */
    map_util_ = map_util;


    /*  local planner intial  */
    if(use_jerk_)
    {
      kinojerk_path_finder_.reset(new KinoJerkAstar);
      kinojerk_path_finder_->setParam(nh);
      kinojerk_path_finder_->init();
      std::cout << "[PlannerManager]: use jerk mode" << std::endl;
    }else{
      kinoacc_path_finder_.reset(new KinoAccAstar);
      kinoacc_path_finder_->setParam(nh);
      kinoacc_path_finder_->init();
      std::cout << "[PlannerManager]: use acc mode" << std::endl;
    }

    /*  visualization intial  */
    visualization_.reset(new opt_planner::PlanningVisualization(nh));


    /*  solver intial  */
    poly_traj_solver_.init(w_total, b_total);

   
    std::cout << "[PlannerManager]: initPlanModules success!" << std::endl;


    poly_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("sikangpolyhedron", 1, true);


  }

  //*************************************local primitives*****************************************//
  //***********************************************************************************************//
  //***********************************************************************************************//
  bool PlannerManager::localPlanner(Eigen::MatrixXd &startState, 
                                    Eigen::MatrixXd &endState)
  { // 3 * 3  [pos, vel, acc]
    // end State could be obtained from previous planned trajs
    Eigen::MatrixXd inner_pts; // (4, N -1)
    Eigen::VectorXd allo_ts;
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<Eigen::Vector3d> path_pts;

    ros::Time time_now = ros::Time::now();
    auto start_timer = std::chrono::high_resolution_clock::now();

    //step two: kinodynamic path searching considering obstacles avoidance
    if (use_jerk_){
      
      if (!kinoPlan(startState, endState, time_now, path_pts, kinojerk_path_finder_))
      {
        std::cout << "[localPlanner]: kinodynamic search fails!" << std::endl;
        return false;
      }

    }else{

      if (!kinoPlan(startState, endState, time_now, path_pts, kinoacc_path_finder_))
      {
        std::cout << "[localPlanner]: kinodynamic search fails!" << std::endl;
        return false;
      }
    }

    auto end_timer = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_timer - start_timer);
    std::cout << "kino plan took"<<duration.count() << "micro sec"<< std::endl;
   std::cout << "[localPlanner]  starting corridor generation..." << std::endl;

    //step three: generating corridor
    if (!getSikangConst(path_pts, inner_pts, allo_ts, hPolys))
    {
      std::cout << "[localPlanner]: corridor generation fails!" << std::endl;
      return false;
    }
    std::cout << "[localPlanner]: optimization..." << std::endl;

    visualization_->displayKinoAStarList(path_pts, Eigen::Vector4d(0.8, 1, 0, 1), 0);
    //step four: unconstrained optimziation
    if (!poly_traj_solver_.minJerkTrajOpt(inner_pts, 
                                          allo_ts,
                                          startState,
                                          endState,
                                          time_now, // ros::Time
                                          hPolys))
    {
      printf("\033[33m [localPlanner]: optimization fails!");
      return false;
    }
  
    //step up local data
    have_opt_path_  = true;
    poly_traj_solver_.getTraj(local_data_.traj_);

    local_data_.start_time_ = time_now;
    local_data_.duration_ = local_data_.traj_.getTotalDuration();
    local_data_.traj_id_ += 1;
    local_data_.start_pos_ = startState.col(0);
    
    visualization_->displayOptimalList(local_data_.traj_.getConstPoints(5), 0);
    std::cout << "\n[localPlanner]: local planning success!" << std::endl;
    return true;
  }

  void PlannerManager::setTraj(std::vector<double> &dura,
                               std::vector<min_jerk::BoundaryCond> &bCond){

    Eigen::VectorXd durs = local_data_.traj_.getDurations();
    dura.clear();
    bCond.clear();

    for (int i = 0; i < durs.size(); ++i)
    {
      dura.push_back(durs(i));
      bCond.push_back(local_data_.traj_[i].getBoundCond());

    }
    return;

  }

  
  // use kinodynamic a* to generate a path and get hpoly
  template <typename T>
  bool PlannerManager::kinoPlan(Eigen::MatrixXd &startState,
                                Eigen::MatrixXd &endState,
                                ros::Time plan_time,
                                std::vector<Eigen::Vector3d> &kino_path,
                                T &finder)
  {
    
    kino_path.clear();
    finder->reset();
    finder->intialMap(map_util_);

    int status = finder->search(startState, endState, plan_time, false);

    if (status == KINO_SEARCH_RESULT::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;
      // retry searching with discontinuous initial state
      finder->reset();
      status = finder->search(startState, endState, plan_time, false);
      if (status == KINO_SEARCH_RESULT::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }
    finder->getKinoTraj(time_res_, kino_path);

    // if ( (kino_path.back() - endState.col(0)).norm() < 1.0)
    // {
    //   //end_state.col(0) = kino_path.back();
    //   kino_path.push_back(endState.col(0));
    // }else{
    //   size_t path_size = kino_path.size()-1;
    //   Eigen::Vector3d end_point = kino_path.back() + 0.01 * ( kino_path.back() -  kino_path[path_size-1]);
    //   kino_path.push_back(end_point);
    //   endState.col(0) = kino_path.back();
    // }
    endState.col(0) = kino_path.back();
    //kino_path.push_back(endState.col(0));


    return true;
  }

  bool PlannerManager::getSikangConst(std::vector<Eigen::Vector3d> &path_pts,
                                      Eigen::MatrixXd &inner_pts,
                                      Eigen::VectorXd &allo_ts,
                                      std::vector<Eigen::MatrixXd> &hPolys)
  {

    hPolys.clear();
    //vec_E<Polyhedron3D> poly_disp;
    Eigen::MatrixXd hPoly;

    std::vector<Eigen::Vector3d> temp_pts;
    std::vector<double> temp_ts;
    std::vector<Eigen::MatrixXd> temp_hPolys;
    //setup the pointcloud
    Vec3f map_size;

    auto dim = map_util_->getDim();
    auto res = map_util_->getRes();

    map_size(0) = res * dim(0);
    map_size(1) = res * dim(1);
    map_size(2) = res * dim(2);

    std::cout << "[PlannerManager]: map_origin is " << map_util_->getOrigin()<< "   map_size is " << map_size << std::endl;
    std::cout << "[PlannerManager]: dim is " << dim << "  res " << res << std::endl;

    // decomp_util_.set_global_bbox(map_util_->getOrigin(), map_size);
    
    
    Vec2f map_vertical_bbx;
    Vec3f origin = map_util_->getOrigin();

    map_vertical_bbx(0) = origin(2);
    map_vertical_bbx(1) = origin(2) + res * dim(2);
    
    decomp_util_.set_vertical_bbox(map_vertical_bbx);

    std::cout << "[PlannerManager]:map_vertical_bbxis " << map_vertical_bbx.transpose() << std::endl;


    decomp_util_.set_obs(map_util_->getCloud());
    decomp_util_.set_local_bbox(Vec3f(4.0, 3.0, 2.0), Vec3f(bb_back_, 3.0, 2.0));

    size_t path_size = path_pts.size();
    Vec3f seed_point1, seed_point2;
    Eigen::Vector3d q;
    int query_index, cnt_num = 0;
    
    
    Eigen::Vector3d end_point = path_pts.back() + 0.01 * ( path_pts.back() -  path_pts[path_size-1]);
    path_pts.push_back(end_point);
    // step 1 : set the intial lengh
    // add start time
    temp_ts.push_back(0.0);

    for (size_t i = 0; i < path_size; i++)
    {
      query_index = i;
      // check wehter or not we need to generate the point
      if (i > 0)
      {
        if (opt_planner::insidehPoly(hPoly,path_pts[query_index]) && cnt_num <= 20)
        {
          //std::cout << "the seed point is inside ! : " << path_pts[query_index] << std::endl;
          cnt_num ++;
          continue;
        }
        else
        {
          //std::cout << "the seed point is not inside ! : " << path_pts[query_index] << std::endl;
          //seed point is not inside
          query_index = i-1;
          cnt_num = 0;
          
        }
        temp_pts.push_back(path_pts[query_index]);
        temp_ts.push_back(query_index * time_res_);
        
        //std::cout << "the query_index * time_res_  is " << query_index * time_res_ << std::endl;
      }
  
      // get one polyhedron

      //std::cout << "the path_pts.at(query_index) is " << path_pts.at(query_index) << std::endl;
      q = path_pts.at(query_index);
      seed_point1 << q(0), q(1), q(2);
      q = path_pts.at(query_index+1);
      seed_point2 << q(0), q(1), q(2);
      vec_Vec3f seed_path;
      seed_path.push_back(seed_point1);
      seed_path.push_back(seed_point2);
      decomp_util_.dilate(seed_path);


      hPoly = decomp_util_.get_hPoly()[0];
      //poly_disp.push_back(decomp_util_.get_polyhedrons()[0]);
      temp_hPolys.push_back(hPoly);
      //std::cout << "hPoly is " << hPoly << std::endl;

    }
    
    size_t final_size = temp_pts.size(); // include the start and end point
    //std::cout << "the final_size is " << final_size << std::endl;


    /***Corridor cut-off and refinements***/
    //1. if only one poly, add it
    if (final_size == 0){
      temp_hPolys.push_back(hPoly);
      int temp_index = path_size/2;
      std::cout << "the path_size is " << path_size << std::endl;
      std::cout << "the temp_index is " << temp_index << std::endl;
      temp_pts.push_back(path_pts[temp_index]);
      temp_ts.push_back( temp_index* time_res_);
      final_size = 1;
    }
    temp_ts.push_back(path_size * time_res_);

    // std::cout << "the temp_ts is " << temp_ts.size() << std::endl;
    // std::cout << "the temp_hPolys.size() is " << temp_hPolys.size() << std::endl;

    //2. delete the overlap corridors
    int M = temp_hPolys.size();
    if (M > 8){

      bool is_overlap;
      std::deque<int> idices;
      idices.push_front(M - 1);
      for (int i = M - 1; i >= 0; i--)
      {
        for (int j = 0; j < i; j++)
        {
          if (j < i - 1)
          {
            is_overlap = overlap(temp_hPolys[i], temp_hPolys[j], 0.01);
          }
          else
          {
            is_overlap = true;
          }
          if (is_overlap)
          {
            if (j < i - 1 && (!opt_planner::insidehPoly(temp_hPolys[i], temp_pts.at(j))))
            {
              continue;
            }
            idices.push_front(j);
            i = j + 1;
            break;

          }
        }
      }

      std::cout << "the idices. is " << idices.size() << std::endl;
      int short_cut_size = idices.size()-1;
      inner_pts.resize(3, short_cut_size);
      allo_ts.setZero(short_cut_size+1);
      hPolys.clear();

      int j = 0;
      int last_ele = 0;
      for (const auto &ele : idices)
      {
        hPolys.push_back(temp_hPolys[ele]);
        allo_ts(j) = temp_ts.at(ele+1) - temp_ts.at(last_ele);
        
        if (j < short_cut_size)
        {
          inner_pts.col(j) =  temp_pts.at(ele);
          //std::cout << "htemp_pts.at(ele) is " << temp_pts.at(ele) << std::endl;
        }
        last_ele = ele+1;
        j +=1;
      }
    }else
    {

      hPolys = temp_hPolys;
      temp_ts.push_back(path_size * time_res_);
      inner_pts.resize(3, final_size);
      allo_ts.setZero(final_size+1);

      for(size_t j = 0; j < final_size; j++ ){

        inner_pts.col(j).head(3) =  temp_pts.at(j);
        allo_ts(j) = temp_ts.at(j+1) - temp_ts.at(j);

      }
      allo_ts(final_size) = temp_ts.at(final_size+1) - temp_ts.at(final_size);
    }

    std::cout << "final_size is " << final_size << std::endl;
    std::cout << "inner_pts is " << inner_pts << std::endl;
    // display
    // decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_disp);
    // poly_msg.header.frame_id = frame_id_;
    // poly_pub_.publish(poly_msg);
    visPoly(hPolys);

    return true;
  }

} // namespace
