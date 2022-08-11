#include <path_searching/kino_jerk_astar.h>

namespace opt_planner
{
  using namespace std;
  using namespace Eigen;

  KinoJerkAstar::~KinoJerkAstar()
  {
    for (int i = 0; i < ksp_.allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  // set params
  void KinoJerkAstar::setParam(ros::NodeHandle &nh)
  {
    nh.param("search/max_tau", ksp_.max_tau_, -1.0);
    nh.param("search/init_max_tau", ksp_.init_max_tau_, -1.0);
    nh.param("search/horizon", ksp_.horizon_, -1.0);

    nh.param("search/resolution_astar", ksp_.resolution_, -1.0);
    nh.param("search/time_resolution", ksp_.time_resolution_, -1.0);

    nh.param("search/lambda_heu", ksp_.lambda_heu_, -1.0);
    nh.param("search/allocate_num", ksp_.allocate_num_, -1);
    nh.param("search/vis_check_num", ksp_.vis_check_num_, -1);

    nh.param("search/w_time", ksp_.w_time_, -1.0);

    nh.param("max_vel",  ksp_.max_vel_, -1.0);
    nh.param("max_acc",  ksp_.max_acc_, -1.0);
    nh.param("max_jerk", ksp_.max_jerk_, -1.0);
  }

  void KinoJerkAstar::intialMap(GridMap::Ptr &grid_map)
  {
    grid_map_ = grid_map;
    ROS_INFO_STREAM("Finish the intialization of the kinodynamic front-end. ");
  }


  void KinoJerkAstar::init(int ego_id, double ego_radius)
  {
    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(ksp_.allocate_num_);
    for (int i = 0; i < ksp_.allocate_num_; i++)
    {
      path_node_pool_[i] = new JerkNode;
    }

    phi_ = Eigen::MatrixXd::Identity(9, 9);
    use_node_num_ = 0;
    iter_num_ = 0;

    double res = ksp_.max_jerk_ * 0.5;
    Eigen::Vector3d um;
    jerk_inputs_.clear();
    for (double jx = -ksp_.max_jerk_; jx <= ksp_.max_jerk_ + 1e-3; jx += res)
      for (double jy = -ksp_.max_jerk_; jy <= ksp_.max_jerk_ + 1e-3; jy += res)
        for (double jz = -ksp_.max_jerk_; jz <= ksp_.max_jerk_ + 1e-3; jz += res)
        {
          um << jx, jy, jz;
          if (um.norm() <= ksp_.max_jerk_ + 1e-3){
            jerk_inputs_.push_back(um);
          }
        }

  }

  int KinoJerkAstar::search(Eigen::MatrixXd startState, // 3 * 3
                            Eigen::MatrixXd endState,  // 3 * 3
                            ros::Time time_start,
                            bool init)
  {
    start_pt_ = startState.col(0);
    start_vel_ = startState.col(1);
    start_acc_ = startState.col(2);
    end_pt_ = endState.col(0);
    end_vel_ = endState.col(1);
    end_acc_ = endState.col(2);
    
    /* ---------- initialize ---------- */
    JerkNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt_;
    cur_node->state.segment(3, 3) = start_vel_;
    cur_node->state.tail(3) = start_acc_;
    cur_node->time = time_start.toSec(); // primitive start time
    cur_node->g_score = 0.0;


    Eigen::Vector3i start_index, end_index;

    grid_map_->posToIndex(start_pt_, start_index);
    cur_node->index = start_index;

    grid_map_->posToIndex(end_pt_, end_index);


    Eigen::VectorXd final_state(9);
    double time_to_goal;

    final_state.head(3) = end_pt_;
    final_state.segment(3, 3) = end_vel_;
    final_state.tail(3) = end_acc_;

    cur_node->f_score = ksp_.lambda_heu_ * estimateHeuristic(cur_node->state, final_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;

    open_set_.push(cur_node);
    use_node_num_ += 1;

    expanded_nodes_.insert(cur_node->index, cur_node);

    //JerkNodePtr neighbor = NULL;
    JerkNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / ksp_.resolution_);
    iter_num_ = 0;


    /* ---------- init state propagation ---------- */
    double time_res = 1 / 1.0, time_res_init = 1 / 8.0;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    Eigen::Vector3d um;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * ksp_.init_max_tau_; tau <= ksp_.init_max_tau_;
            tau += time_res_init * ksp_.init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      for (double tau = time_res * ksp_.max_tau_; tau <= ksp_.max_tau_; tau += time_res * ksp_.max_tau_)
        durations.push_back(tau);
    }

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      /* ---------- get lowest f_score node ---------- */
      cur_node = open_set_.top();

      /* ---------- determine termination ---------- */
      bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                      abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                      abs(cur_node->index(2) - end_index(2)) <= tolerance;
      // to decide the near end

      bool reach_horizon = (cur_node->state.head(3) - start_pt_).norm() >= ksp_.horizon_;

      if (reach_horizon || near_end)
      {
        terminate_node = cur_node;
        retrievePath(terminate_node, path_nodes_);
        has_path_ = true;

        
        if (near_end)
        {
          cout << "[Kino Astar]: near end." << endl;

          /* one shot trajectory */
          estimateHeuristic(cur_node->state, final_state, time_to_goal);
          computeShotTraj(cur_node->state, final_state, time_to_goal);
          
          if (terminate_node->parent == NULL && !is_shot_succ_)
          {
            return NO_PATH;
          }
          else if (!is_shot_succ_)
          {
            cout << "[Kino Astar]: Reach end but shot fails!" << endl;
            return KINO_SEARCH_RESULT::REACH_END_BUT_SHOT_FAILS;
          }
          else
          {
            cout << "[Kino Astar]: reach  end." << endl;
            return KINO_SEARCH_RESULT::REACH_END;
          }
        }
        else if (reach_horizon)
        {
          cout << "[Kino Astar]: Reach horizon !" << endl;
          return KINO_SEARCH_RESULT::REACH_HORIZON;
        }
      }

      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;


      Eigen::Matrix<double, 9, 1> cur_state = cur_node->state, pro_state;
      //Eigen::Vector2d cur_yaw = cur_node->yaw_state, pro_yaw;

      vector<JerkNodePtr> tmp_expand_nodes;

      double pro_start_time;
      //std::cout << " cur_state " << cur_state << std::endl;
    
      /* ---------- state propagation loop ---------- */
      for (unsigned int j = 0; j < durations.size(); ++j)
      {
        double tau = durations[j];
        pro_start_time = cur_node->time + tau;
        double vis_res = tau / double(ksp_.vis_check_num_);

        for (unsigned int i = 0; i < inputs.size(); ++i)
        {
          init_search = false;
          um = inputs[i]; // jerk
          bool is_feasible = true;

          stateTransit(cur_state, pro_state, um, tau);

          /* ---------- check if in free space ---------- */

          /* inside map range */
          Eigen::Vector3d pro_pos = pro_state.head(3);
          if (!grid_map_->isInMap(pro_pos))
          {
            continue;
          }

          /* not in close set */
          Eigen::Vector3i pro_id;
          grid_map_->posToIndex(pro_pos, pro_id);
          JerkNodePtr pro_node = expanded_nodes_.find(pro_id);

          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            continue;
          }
          
          /* not in the same voxel */
          Eigen::Vector3i diff = pro_id - cur_node->index;
          if (diff.norm() == 0)
          {
            continue;
          }


          /* acc feasibe */
          Eigen::Vector3d pro_acc = pro_state.tail(3);
          if (fabs(pro_acc(0)) > ksp_.max_acc_ || fabs(pro_acc(1)) > ksp_.max_acc_ || fabs(pro_acc(2)) > ksp_.max_acc_)
          {
            if (init_search)
              std::cout << "acc is" << pro_acc << std::endl;
            continue;
          }

          /* vel feasibe   v = 0.5 * j * t^2 + a0 t + v0  and v0 is feasible*/
          // 0.5 * j * t^2 + a t + v0  = v_max 
          // t =  
          //Eigen::Vector3d pro_vel = pro_state.segment(3, 3);
          //TrajDim, TrajOrder  normalized posCoeff = [c3*T^3,c2*T^2,c1*T,c0*1]
          Eigen::MatrixXd velCoeff(3, 3);
          velCoeff.col(0) = 0.5 * um;
          velCoeff.col(1) = cur_state.tail(3);
          velCoeff.col(2) = cur_state.segment(3, 3);

          for (int i = 0; i < 3; i++)
          {
            Eigen::VectorXd coeff = RootFinder::polySqr(velCoeff.row(i));
            coeff.tail<1>()(0) -= (ksp_.max_vel_+ 1e-2) * (ksp_.max_vel_+ 1e-2);
            if (RootFinder::countRoots(coeff, 0.0, tau) > 0 ){
              //cout << "vel infeasible" << endl;
              continue;
              is_feasible = false;
            }
          }
          if (!is_feasible)
            continue;

        
          /* collision free */
          Eigen::Matrix<double, 9, 1> xt;

          for (int k = 1; k <= ksp_.vis_check_num_; ++k)
          {
            double dt = vis_res * double(k);
            stateTransit(cur_state, xt, um, dt);
            
            if (grid_map_->getInflateOccupancy(xt.head(3)))
            {
              is_feasible = false;
              //cout << "collide" << endl;
              break;
            }
          }

          if (!is_feasible)
            continue;

          
          /* ---------- compute cost ---------- */
          double time_to_goal, tmp_g_score, tmp_f_score;

          
          tmp_g_score = (um.squaredNorm() + ksp_.w_time_ ) * tau  + cur_node->g_score;
          tmp_f_score = tmp_g_score + ksp_.lambda_heu_ * estimateHeuristic(pro_state, final_state, time_to_goal);

          /* ---------- compare expanded node in this loop ---------- */
          bool prune = false;
          for (unsigned int j = 0; j < tmp_expand_nodes.size(); ++j)
          {
            JerkNodePtr expand_node = tmp_expand_nodes[j];
            if ((pro_id - expand_node->index).norm() == 0)
            {
              prune = true;
              if (tmp_f_score < expand_node->f_score)
              {
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->state = pro_state;
                expand_node->input = um;
                expand_node->duration = tau;
                expand_node->time = pro_start_time;
              }
              break;
            }
          }

          /* ---------- new neighbor in this loop ---------- */
          if (!prune)
          {
            if (pro_node == NULL)
            {
              pro_node = path_node_pool_[use_node_num_];
              pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->node_state = IN_OPEN_SET;
              pro_node->time = pro_start_time;

              open_set_.push(pro_node);
              expanded_nodes_.insert(pro_id, pro_node);
              tmp_expand_nodes.push_back(pro_node);

              use_node_num_ += 1;
              if (use_node_num_ == ksp_.allocate_num_)
              {
                cout << "run out of memory." << endl;
                
                return NO_PATH;
              }
            }
            else if (pro_node->node_state == IN_OPEN_SET)
            {
              if (tmp_g_score < pro_node->g_score)
              {
                // pro_node->index = pro_id;
                pro_node->state = pro_state;
                pro_node->f_score = tmp_f_score;
                pro_node->g_score = tmp_g_score;
                pro_node->input = um;
                pro_node->duration = tau;
                pro_node->parent = cur_node;

              }
            }
            else
            {
              cout << "error type in searching: " << pro_node->node_state << endl;
            }
          }

        } /* ----------  ---------- */
      }
    }

    cout << "[Kino Astar]: open set empty, no path." << endl;
    /* ---------- open set empty, no path ---------- */
    return NO_PATH;
  }


  // need to revise
  double KinoJerkAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                             double &optimal_time)
  {
    const Vector3d dp = x2.head(3) - x1.head(3);
    const Vector3d v0 = x1.segment(3, 3);
    const Vector3d v1 = x2.segment(3, 3);
    const Vector3d a0 = x1.tail(3);
    const Vector3d a1 = x2.tail(3);

    double c1 = -720* dp.dot(dp);
    double c2 =  720 * (v0 + v1).dot(dp);
    double c3 = -192 * (v0.dot(v0) + v1.dot(v1)) + 336 * v0.dot(v1)  + 120 * (a0 - a1).dot(dp);
    double c4 = -48 * a0.dot(v1) + 48 * a1.dot(v0) - 72 * a0.dot(v0) + 72 * a1.dot(v1);
    double c5 =  6 * a0.dot(a1) - 9 * a1.dot(a1) - 9 * a0.dot(a0);

    std::vector<double> ts = solve(ksp_.w_time_, 0.0, 0.0, c5, c4, c3, c2, c1);

    double v_max = ksp_.max_vel_;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
      if (t < t_bar)
        continue;

      double t_2 = t * t;
      double t_3 = t_2 * t;
      double t_4 = t_3 * t;

      double c = -c1 / (6 * t_3 * t_3) - c2 / (5 * t_3 * t_2) - c3 / (4 * t_4) - c4 / (3 * t_3) - c5 / (2 * t_2) + ksp_.w_time_ * t;
      if (c < cost)
      {
        cost = c;
        t_d = t;
      }
    }
    optimal_time = t_d;
    return 1.0 * (2.0 + 1.0 / 10000) * cost;
  }

  // need to revise
  bool KinoJerkAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                         double time_to_goal)
  {
    /* ---------- get coefficient ---------- */
    const Vector3d p0 = state1.head(3);
    const Vector3d dp = state2.head(3) - p0;
    const Vector3d v0 = state1.segment(3, 3);
    const Vector3d dv = state2.segment(3, 3) - v0;
    const Vector3d a0 = state1.tail(3);

    // order = 5;
    double t_d = time_to_goal;
    MatrixXd coef(3, 6);

    double t_d_2 = t_d * t_d;
    double t_d_3 = t_d_2 * t_d;
    double t_d_4 = t_d_3 * t_d;

    const Vector3d delta_p = dp - v0 * t_d - 0.5 * a0 * t_d_2;
    const Vector3d delta_v = dv - a0 * t_d;
    const Vector3d delta_a = state2.tail(3) - a0;

    Vector3d a = 1.0 / 120.0 * (720.0 / (t_d_4 * t_d) * delta_p - 360.0 / t_d_4 * delta_v + 60.0 / t_d_3 * delta_a);
    Vector3d b = 1.0 / 24.0 * (-360.0 / t_d_4 * delta_p + 168.0 / t_d_3 * delta_v - 24.0 / t_d_2 * delta_a);
    Vector3d c = 1.0 / 6.0 * (60.0 / t_d_3 * delta_p - 24.0 / t_d_2 * delta_v + 3.0 / t_d * delta_a);
    Vector3d d = 0.5 * a0;
    Vector3d e = v0;
    Vector3d f = p0;

    // 1/24 *  * t^4  +  1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^4 + b*t^3 + c*t^2 + v0*t + p0
    coef.col(5) = a;
    coef.col(4) = b;
    coef.col(3) = c;
    coef.col(2) = d;
    coef.col(1) = e;
    coef.col(0) = f;

    Vector3d coord, vel, acc, jerk;
    VectorXd poly1d, t, polyv, polya;
    Vector3i index;

    Eigen::MatrixXd Tm(6, 6);
    Tm << 0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        0, 0, 0, 3, 0, 0,
        0, 0, 0, 0, 4, 0,
        0, 0, 0, 0, 0, 5,
        0, 0, 0, 0, 0, 0;

    /* ---------- forward checking of trajectory ---------- */
    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta)
    {
      t = VectorXd::Zero(6);
      for (int j = 0; j < 6; j++)
        t(j) = pow(time, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef.row(dim);
        coord(dim) = poly1d.dot(t);
        vel(dim) = (Tm * poly1d).dot(t);
        acc(dim) = (Tm * Tm * poly1d).dot(t);
        jerk(dim) = (Tm * Tm * Tm * poly1d).dot(t);

        if (fabs(vel(dim)) > ksp_.max_vel_ || fabs(vel(dim)) > ksp_.max_vel_)
        {
          // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
          // return false;
        }

        if (fabs(acc(dim)) > ksp_.max_acc_ || fabs(acc(dim)) > ksp_.max_acc_)
        {
          // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
          // return false;
        }

      }

      if (grid_map_->getInflateOccupancy(coord))
      {
        return false;
      }
    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true;
    return true;
  }


  void KinoJerkAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<JerkNodePtr, std::vector<JerkNodePtr>, NodeComparator<JerkNodePtr>> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      JerkNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
  }


  // call after the trajectory is successfully generated
  void KinoJerkAstar::getKinoTraj(double delta_t,
                                  std::vector<Eigen::Vector3d> &path_list)
  {
    path_list.clear();

    /* ---------- get traj of searching ---------- */
    JerkNodePtr node = path_nodes_.back();
    Matrix<double, 9, 1> x0, xt;

    while (node->parent != NULL)
    {
      Vector3d ut = node->input;
      double duration = node->duration;
      x0 = node->parent->state;

      for (double t = duration; t >= -1e-5; t -= delta_t)
      {
        stateTransit(x0, xt, ut, t);

        path_list.push_back(xt.head(3));
  
      }
      node = node->parent;
    }
    reverse(path_list.begin(), path_list.end());

    

    /* ---------- get traj of one shot ---------- */
    if (is_shot_succ_)
    {
      Vector3d coord;
      VectorXd poly1d, time(6);

      for (double t = delta_t; t <= t_shot_; t += delta_t)
      {
        for (int j = 0; j < 6; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++)
        {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }

        if (path_list.size() < 1 || (path_list.back().head(3) - coord).norm() > 0.0)
        {
          path_list.push_back(coord);

        }
      }
    }

    return;
  }

  void KinoJerkAstar::stateTransit(Eigen::Matrix<double, 9, 1> &state0,
                                      Eigen::Matrix<double, 9, 1> &state1, Eigen::Vector3d um,
                                      double tau)
  {
    for (int i = 0; i < 6; ++i)
      phi_(i, i + 3) = tau;

    Eigen::Matrix<double, 9, 1> integral;

    integral.head(3) = 1.0 / 6.0 * pow(tau, 3) * um;
    integral.segment(3, 3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;


    state1 = phi_ * state0 + integral;
  }

} // namespace
