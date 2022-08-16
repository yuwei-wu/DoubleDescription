// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <visualization_msgs/Marker.h>

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

   std::cout << "[localPlanner]: corridor generation..." << std::endl;

    //step three: generating corridor
    if (!getSikangConst(path_pts, inner_pts, allo_ts, hPolys))
    {
      std::cout << "[localPlanner]: corridor generation fails!" << std::endl;
      return false;
    }
    std::cout << "[localPlanner]: optimization..." << std::endl;


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
    //endState.col(0) = kino_path.back();
    visualization_->displayKinoAStarList(kino_path, Eigen::Vector4d(0.8, 1, 0, 1), 0);
    return true;
  }

  bool PlannerManager::getSikangConst(std::vector<Eigen::Vector3d> &path_pts,
                                      Eigen::MatrixXd &inner_pts,
                                      Eigen::VectorXd &allo_ts,
                                      std::vector<Eigen::MatrixXd> &hPolys)
  {

    hPolys.clear();
    vec_E<Polyhedron3D> poly_disp;
    Eigen::MatrixXd hPoly;

    std::vector<Eigen::Vector3d> temp_pts;
    std::vector<double> temp_ts;
    
    //setup the pointcloud
    Vec3f map_size;

    auto dim = map_util_->getDim();
    auto res = map_util_->getRes();

    map_size(0) = res * dim(0);
    map_size(1) = res * dim(1);
    map_size(2) = res * dim(2);

    std::cout << "[PlannerManager]: map_origin is " << map_util_->getOrigin()<< "   map_size is " << map_size << std::endl;
    std::cout << "[PlannerManager]: dim is " << dim << "  res " << res << std::endl;

    decomp_util_.set_global_bbox(map_util_->getOrigin(), map_size);
    decomp_util_.set_obs(map_util_->getCloud());
    decomp_util_.set_local_bbox(Vec3f(4.0, 3.0, 2.0), Vec3f(bb_back_, 3.0, 2.0));

    size_t path_size = path_pts.size();
    Vec3f seed_point1, seed_point2;
    Eigen::Vector3d q;
    int query_index, cnt_num = 0;
    
    
    Eigen::Vector3d end_point = path_pts.back() + 0.1 * ( path_pts.back() -  path_pts[path_size-1]);
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
        if (poly_disp.back().inside(path_pts[query_index]) && cnt_num <= 15)
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
      poly_disp.push_back(decomp_util_.get_polyhedrons()[0]);
      hPolys.push_back(hPoly);
      //std::cout << "hPoly is " << hPoly << std::endl;

    }

    // display
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_disp);
    poly_msg.header.frame_id = frame_id_;
    poly_pub_.publish(poly_msg);

    
    size_t final_size = temp_pts.size(); // include the start and end point
    //std::cout << "the final_size is " << final_size << std::endl;

    if (final_size == 0){
      hPolys.push_back(hPoly);
      int temp_index = path_size/2;
      std::cout << "the path_size is " << path_size << std::endl;
      std::cout << "the temp_index is " << temp_index << std::endl;
      temp_pts.push_back(path_pts[temp_index]);
      temp_ts.push_back( temp_index* time_res_);
      final_size = 1;
    }

    temp_ts.push_back(path_size * time_res_);
    inner_pts.resize(3, final_size);
    allo_ts.setZero(final_size+1);
 

    for(size_t j = 0; j < final_size; j++ ){

      inner_pts.col(j).head(3) =  temp_pts.at(j);
      allo_ts(j) = temp_ts.at(j+1) - temp_ts.at(j);

    }
    allo_ts(final_size) = temp_ts.at(final_size+1) - temp_ts.at(final_size);
    

    //std::cout << "inner_pts is " << inner_pts << std::endl;

    return true;
  }

} // namespace
