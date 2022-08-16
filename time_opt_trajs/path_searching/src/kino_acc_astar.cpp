#include <path_searching/kino_acc_astar.h>


namespace opt_planner
{
  using namespace std;
  using namespace Eigen;

  KinoAccAstar::~KinoAccAstar()
  {
    for (int i = 0; i < ksp_.allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  // set params
  void KinoAccAstar::setParam(ros::NodeHandle &nh)
  {
    nh.param("search/max_tau", ksp_.max_tau_, -1.0);
    nh.param("search/init_max_tau", ksp_.init_max_tau_, -1.0);
    nh.param("search/horizon", ksp_.horizon_, -1.0);
    nh.param("search/resolution_astar", ksp_.resolution_, -1.0);
    nh.param("search/lambda_heu", ksp_.lambda_heu_, -1.0);
    nh.param("search/allocate_num", ksp_.allocate_num_, -1);
    nh.param("search/vis_check_num", ksp_.vis_check_num_, -1);
    nh.param("search/w_time", ksp_.w_time_, -1.0);

    nh.param("max_v",  ksp_.max_vel_, -1.0);
    nh.param("max_a",  ksp_.max_acc_, -1.0);
    nh.param("max_j", ksp_.max_jerk_, -1.0);

  }

  void KinoAccAstar::intialMap(const std::shared_ptr<MPL::VoxelMapUtil> &map_util)
  {
    kino_map_util_ = map_util;
  }


  void KinoAccAstar::init()
  {
    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(ksp_.allocate_num_);
    for (int i = 0; i < ksp_.allocate_num_; i++)
    {
      path_node_pool_[i] = new AccNode;
    }

    phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;

    // set up the inputs
    Eigen::Vector3d um;
    double res = ksp_.max_acc_ * 0.5;
    acc_inputs_.clear();
    for (double ax = -ksp_.max_acc_; ax <= ksp_.max_acc_ + 1e-3; ax += res)
      for (double ay = -ksp_.max_acc_; ay <= ksp_.max_acc_ + 1e-3; ay += res)
        for (double az = -ksp_.max_acc_; az <= ksp_.max_acc_ + 1e-3; az += res)
        {
          um << ax, ay, az;
          acc_inputs_.push_back(um);
        }
    
  }

  int KinoAccAstar::search(Eigen::MatrixXd startState, // 3 * 2
                           Eigen::MatrixXd endState,  // 3 * 2
                           ros::Time time_start,
                           bool init)
  {
    start_pt_ = startState.col(0);
    start_vel_ = startState.col(1);
    end_pt_ = endState.col(0);
    end_vel_ = endState.col(1);
    /* ---------- initialize ---------- */
    AccNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt_;
    cur_node->state.tail(3) = start_vel_;
    cur_node->time = time_start.toSec(); // primitive start time
    cur_node->g_score = 0.0;

    Vec3i start_id = kino_map_util_->floatToInt(Vec3f(start_pt_(0), start_pt_(1), start_pt_(2)));
    Vec3i end_id   = kino_map_util_->floatToInt(Vec3f(end_pt_(0), end_pt_(1), end_pt_(2)));

    Eigen::Vector3i start_index(start_id(0), start_id(1), start_id(2)) ;
    Eigen::Vector3i end_index(end_id(0), end_id(1), end_id(2)) ; 

    cur_node->index = start_index;

    Eigen::VectorXd final_state(6);
    double time_to_goal;

    final_state.head(3) = end_pt_;
    final_state.tail(3) = end_vel_;
  
    cur_node->f_score = ksp_.lambda_heu_ * estimateHeuristic(cur_node->state, final_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;

    open_set_.push(cur_node);
    use_node_num_ += 1;

    expanded_nodes_.insert(cur_node->index, cur_node);

    AccNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / ksp_.resolution_);
    iter_num_ = 0;

    /* ---------- init state propagation ---------- */
    double time_res = 1 / 1.0, time_res_init = 1 / 8.0;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * ksp_.init_max_tau_; tau <= ksp_.init_max_tau_;
            tau += time_res_init * ksp_.init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      inputs = acc_inputs_;
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

      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state, pro_state;
      vector<AccNodePtr> tmp_expand_nodes;
      double pro_start_time;
      Eigen::Vector3d um;
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

          stateTransit(cur_state, pro_state, um, tau);

          //std::cout << "[KinoAccAstar::search]:  pro_state " <<   pro_state  << std::endl;


          /* ---------- check if in free space ---------- */

          /* inside map range */
          Eigen::Vector3d pro_pos = pro_state.head(3);
          Vec3f pos;
          pos << pro_pos(0), pro_pos(1), pro_pos(2);

          Vec3i pn = kino_map_util_->floatToInt(pos);

          if (kino_map_util_->isOutside(pn))
          {
            continue;
          }

          /* not in close set */
          Eigen::Vector3i pro_id(pn(0), pn(1), pn(2)) ;

          AccNodePtr pro_node = expanded_nodes_.find(pro_id);

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

          /* Check maximal velocity */
          Eigen::Vector3d pro_v = pro_state.tail(3);
          if (fabs(pro_v(0)) > ksp_.max_vel_ || fabs(pro_v(1)) > ksp_.max_vel_ || fabs(pro_v(2)) > ksp_.max_vel_)
          {
            if (init_search)
              std::cout << "vel is" << pro_v << std::endl;
            continue;
          }
          
          /* collision free */
          Eigen::Matrix<double, 6, 1> xt;
          bool is_occ = false;

          for (int k = 1; k <= ksp_.vis_check_num_; ++k)
          {
            double dt = vis_res * double(k);
            stateTransit(cur_state, xt, um, dt);

            pos << xt(0), xt(1), xt(2);

            pn = kino_map_util_->floatToInt(pos);

            if (kino_map_util_->isOccupied(pn))
            {
              is_occ = true;
              //cout << "collide" << endl;
              break;
            }

          }
          if (is_occ)
          {
            if (init_search)
              std::cout << "safe" << std::endl;
            continue;
          }
          /* ---------- compute cost ---------- */
          double time_to_goal, tmp_g_score, tmp_f_score;

          
          tmp_g_score = (um.squaredNorm() + ksp_.w_time_ ) * tau  + cur_node->g_score;
          tmp_f_score = tmp_g_score + ksp_.lambda_heu_ * estimateHeuristic(pro_state, final_state, time_to_goal);

          /* ---------- compare expanded node in this loop ---------- */
          bool prune = false;
          for (unsigned int j = 0; j < tmp_expand_nodes.size(); ++j)
          {
            AccNodePtr expand_node = tmp_expand_nodes[j];
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
  double KinoAccAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                             double &optimal_time)
  {
    const Vector3d dp = x2.head(3) - x1.head(3);
    const Vector3d v0 = x1.segment(3, 3);
    const Vector3d v1 = x2.segment(3, 3);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = ksp_.w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = ksp_.max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
      if (t < t_bar)
        continue;
      double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + ksp_.w_time_ * t;
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
  bool KinoAccAstar::computeShotTraj(Eigen::VectorXd state1, 
                                     Eigen::VectorXd state2,
                                     double time_to_goal)
  {
    /* ---------- get coefficient ---------- */
    const Vector3d p0 = state1.head(3);
    const Vector3d dp = state2.head(3) - p0;
    const Vector3d v0 = state1.segment(3, 3);
    const Vector3d v1 = state2.segment(3, 3);
    const Vector3d dv = v1 - v0;
    double t_d = time_to_goal;
    MatrixXd coef(3, 4);
    end_vel_ = v1;

    Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    Vector3d c = v0;
    Vector3d d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

    Vector3d coord, vel, acc;
    VectorXd poly1d, t, polyv, polya;
    Vector3i index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

    /* ---------- forward checking of trajectory ---------- */
    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta)
    {
      t = VectorXd::Zero(4);
      for (int j = 0; j < 4; j++)
        t(j) = pow(time, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef.row(dim);
        coord(dim) = poly1d.dot(t);
        vel(dim) = (Tm * poly1d).dot(t);
        acc(dim) = (Tm * Tm * poly1d).dot(t);

        if (fabs(vel(dim)) > ksp_.max_vel_ || fabs(acc(dim)) > ksp_.max_acc_)
        {
          // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
          // return false;
        }
      }

      Vec3f pos;
      pos << coord(0), coord(1), coord(2);

      auto pn = kino_map_util_->floatToInt(pos);


      /* within the map */
      if (kino_map_util_->isOutside(pn))
      {
        return false;
      }

      if (kino_map_util_->isOccupied(pn))
      {
        return false;
      }
    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true;
    return true;
  }

  void KinoAccAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<AccNodePtr, std::vector<AccNodePtr>, NodeComparator<AccNodePtr>> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      AccNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
  }


  // call after the trajectory is successfully generated
  void KinoAccAstar::getKinoTraj(double delta_t,
                                std::vector<Eigen::Vector3d> &path_list)
  {
    path_list.clear();

    /* ---------- get traj of searching ---------- */
    AccNodePtr node = path_nodes_.back();
    Matrix<double, 6, 1> x0, xt;

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
      VectorXd poly1d, time(4);

      for (double t = delta_t; t <= t_shot_; t += delta_t)
      {
        for (int j = 0; j < 4; j++)
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

  void KinoAccAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0,
                                  Eigen::Matrix<double, 6, 1> &state1, 
                                  Eigen::Vector3d um,
                                  double tau)
  {
    for (int i = 0; i < 3; ++i)
      phi_(i, i + 3) = tau;

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi_ * state0 + integral;
  }

} // namespace
