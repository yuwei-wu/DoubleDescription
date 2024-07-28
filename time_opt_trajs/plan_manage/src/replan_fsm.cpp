
#include <plan_manage/replan_fsm.h>
#include <kr_tracker_msgs/Transition.h>
#include <std_srvs/Trigger.h>

namespace opt_planner
{
 

  void ReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      nh.param("fsm/waypoint_num", waypoint_num_, -1);
      for (int i = 0; i < waypoint_num_; i++)
      {
        nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
      }
    }

    nh.param("initial_offset/x", initial_offset_(0), 0.0);
    nh.param("initial_offset/y", initial_offset_(1), 0.0);

    nh.param("multi_comm_range", multi_comm_range_, 3.0);
    
    
    nh.param("forward_dist", forward_dist_, 2.0);
    std::cout << "forward_dist_ is " << forward_dist_ << std::endl;


    nh.param("poly_srv_name", poly_srv_name_, std::string(" "));

    initial_offset_(2) = 0.0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new PlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &ReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.02), &ReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &ReplanFSM::odometryCallback, this);
    waypoint_sub_ = nh.subscribe("waypoints", 1, &ReplanFSM::waypointCallback, this);

    //trajectory boardcast
    traj_goal_pub_ = nh.advertise<kr_tracker_msgs::PolyTrackerActionGoal>("tracker_cmd", 10);

    //dodge trigger
    trigger_sub_ = nh.subscribe("trigger", 1, &ReplanFSM::triggerCallback, this);
    max_forward_dist_ = planner_manager_->grid_map_->getMapSize()(0) +
                        planner_manager_->grid_map_->getOrigin()(0) - 1.0;

    // //dodge api
    // cmd_pub_ = nh.advertise<kr_tracker_msgs::PolyTrackerActionGoal>("tracker_cmd", 10);
    
  }

  void ReplanFSM::triggerCallback(const std_msgs::EmptyConstPtr &msg)
  {
    if (abs(odom_pos_(0) - max_forward_dist_) < 1.0)
    {
      return;
    }


    trigger_ = true;
    have_target_ = true;
    waypoint_num_ = 1;
    std::cout << "triggered new goal" << std::endl;

    //set a waypoint in front of the drone
    waypoints_[0][0] = std::min(odom_pos_(0) + forward_dist_, max_forward_dist_);
    waypoints_[0][1] = odom_pos_(1);
    waypoints_[0][2] = 1.5;

    

    current_wp_ = 0;
    setGoal();
  
  }

  void ReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {

    if (msg->poses.size() < 1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_num_ = msg->poses.size();
      for (int i = 0; i < waypoint_num_; i++)
      {
        waypoints_[i][0] = msg->poses[i].pose.position.x;
        waypoints_[i][1] = msg->poses[i].pose.position.y;
        waypoints_[i][2] = 0.6;
      }
    }

    current_wp_ = 0;
    setGoal();
  }

  // to prevent the goal is too close to the odom

  bool ReplanFSM::setGoal()
  {

    if (current_wp_ >= waypoint_num_)
    {
      have_target_ = false; // finish all the waypoints
      return false;
    }

    while (current_wp_ < waypoint_num_)
    {
      ROS_INFO_STREAM("waypoint_num_  is : " << waypoint_num_);
      ROS_INFO_STREAM("current_wp_ index  is : " << current_wp_);
      end_pt_(0) = waypoints_[current_wp_][0];
      end_pt_(1) = waypoints_[current_wp_][1];
      end_pt_(2) = waypoints_[current_wp_][2];
      ROS_INFO_STREAM(" end_pt_ is : " << end_pt_);
      current_wp_ += 1;
      if ((end_pt_ - odom_pos_).norm() >= 0.3)
      {
        end_vel_.setZero();
        have_target_ = true;
        return true;
      }
    }

    return false;
  }

  void ReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    odom_yaw_ = atan2(2.0 * (odom_orient_.w() * odom_orient_.z() + odom_orient_.x() * odom_orient_.y()),
                1.0 - 2.0 * (odom_orient_.y() * odom_orient_.y() + odom_orient_.z() * odom_orient_.z()));


    have_odom_ = true;
  }

  void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "INIT_YAW", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, ReplanFSM::FSM_EXEC_STATE> ReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void ReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "INIT_YAW", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    //cout << "ReplanFSM::execFSMCallback" << endl;
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 200)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      //std::cout << "-----------------WAIT_TARGET" << std::endl;
      if (!have_target_)
      {
        //std::cout << "no target, odom is " << odom_pos_ << std::endl;
        return;
      }
      else
      {
        std::cout << "have target" << std::endl;
        local_plan_fail_cnt_   = 0;
        std::cout << "setYaw" << std::endl;
        //setYaw();
        
        //check if the target is too close to the odom
        if ((end_pt_ - odom_pos_).norm() < 0.3)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }

        planner_manager_->setGlobalGoal(end_pt_);
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        init_yaw_time_ = ros::Time::now();
      }
      break;
    }

    case INIT_YAW:
    {

      double dyaw = desired_yaw_ - odom_yaw_;

      if (dyaw > M_PI){
        dyaw = 2 * M_PI - dyaw;
      }
      else if (dyaw <= -M_PI)
      {
        dyaw = dyaw + 2 * M_PI;
      }

      if (abs(dyaw) < 0.6 || (ros::Time::now() - init_yaw_time_).toSec() > 2.0)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }


      break;
    }

    case GEN_NEW_TRAJ:
    {

      Eigen::MatrixXd startState(3, 3);
      Eigen::Vector3d startYawState;
      startState << odom_pos_, odom_vel_, Eigen::MatrixXd::Zero(3, 1);
      startYawState  << odom_yaw_, 0.0, 0.0;

      std::cout << "startState  "  << startState << std::endl;
      if (planner_manager_->localPlanner(startState, startYawState))
      {
        publishTraj();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        if (local_plan_fail_cnt_> 20){
          have_target_ = false;
          //ROS_WARN("the kinodynamic planning fails so many times, please try other ...");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }else{
          local_plan_fail_cnt_ += 1;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {


      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      Eigen::MatrixXd startState(3, 3);
      startState << info->traj_.getPos(t_cur), info->traj_.getVel(t_cur), info->traj_.getAcc(t_cur);

      Eigen::Vector3d startYawState;
      //startYawState  << info->traj_.getYaw(t_cur), info->traj_.getdYaw(t_cur);
      startYawState  << odom_yaw_, 0.0, 0.0;

      if (planner_manager_->localPlanner(startState, startYawState))
      {
        publishTraj();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        cout << "replanning fails, need to replan it" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);
      //cout << "[FSM] t_cur " << t_cur << endl;

      Eigen::Vector3d pos = info->traj_.getPos(t_cur);
      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2 || (pos - end_pt_).norm() < 1e-3)
      {
        ROS_INFO_STREAM("current time larger than duration");
        setGoal();
        changeFSMExecState(WAIT_TARGET, "FSM");

      }
      // else if((end_pt_ - pos).norm() < no_replan_thresh_){

      // }
      else if (t_cur > 0.5)
      {
        cout << "[FSM] from exec to replan t_cur " << t_cur << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      else if ((odom_pos_ - pos).norm() > 2.0)
      {
        cout << "[FSM] The agent does not follow the trajectory, stop! " << endl;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        planner_manager_->EmergencyStop(odom_pos_);
        publishTraj();
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }
  }

  void ReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    //cout << "ReplanFSM::checkCollisionCallback" << endl;
    if (exec_state_ == WAIT_TARGET)
      return;
    auto map = planner_manager_->grid_map_;

    // step one: check the target
    // 1. the final target is or not collision free
    if (have_target_)
    {
      //std::cout << "check whether the final target is collision free end_pt_" << end_pt_ << std::endl;
      if (map->getInflateOccupancy(end_pt_))
      {
        //std::cout << "try to find a max distance goal around " << std::endl;
        /* try to find a max distance goal around */
        const double dr = 0.2, dtheta = 30, dz = 0.2;
        double new_x, new_y, new_z;
        Eigen::Vector3d goal;

        for (double r = dr; r <= 8 * dr + 1e-3; r += dr)
        {
          for (double theta = -90; theta <= 270; theta += dtheta)
          {
            for (double nz = 0.5; nz <= 2.0; nz += dz)
            {

              new_x = end_pt_(0) + r * cos(theta);
              new_y = end_pt_(1) + r * sin(theta);
              new_z = nz;

              Eigen::Vector3d new_pt(new_x, new_y, new_z);
              if (!map->getInflateOccupancy(new_pt))
              {
                end_pt_ = new_pt;
                planner_manager_->setGlobalGoal(end_pt_);
                ROS_WARN("the global target is not collision free, replan.");
                changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                break;
              }
            }
          }
        }
      }
    }

    /* ---------- check trajectory ---------- */
    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.toSec() < 1e-5)
      return;

    constexpr double time_step = 0.02;
    //double global_t_cut = ros::Time::now().toSec();
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      auto pos = info->traj_.getPos(t);
      if (map->getInflateOccupancy(pos))
      {
        if (t - t_cur < emergency_time_) // 0.8s of emergency time
        {
          std::cout << " the pos is " << pos << "time is " << t_cur << std::endl;
          ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }

    }

  }


  bool ReplanFSM::publishTraj()
  {
    auto info = &planner_manager_->local_data_;
    int order = 5;

    // publish the message for multi-agent connections and for trajectory server

    kr_tracker_msgs::PolyTrackerActionGoal traj_act_msg;

    //traj_act_msg.goal.frame_id = frame_id_;
    traj_act_msg.goal.order = order;
    traj_act_msg.goal.set_yaw = false;


    Eigen::VectorXd durs = info->traj_.getDurations();
    int piece_num = info->traj_.getPieceNum();

    traj_act_msg.goal.t_start = info->start_time_;
    traj_act_msg.goal.seg_x.resize(piece_num);
    traj_act_msg.goal.seg_y.resize(piece_num);
    traj_act_msg.goal.seg_z.resize(piece_num);

    std::cout << "!!! durs" << durs << std::endl;


    for (int i = 0; i < piece_num; ++i)
    {
      auto coeff = info->traj_[i].getNormalizedCoeffMat();

      for (uint c = 0; c <= order; c++) {
        traj_act_msg.goal.seg_x[i].coeffs.push_back(coeff(0,order-c));
        traj_act_msg.goal.seg_y[i].coeffs.push_back(coeff(1,order-c));
        traj_act_msg.goal.seg_z[i].coeffs.push_back(coeff(2,order-c));
      }
      traj_act_msg.goal.seg_x[i].dt = durs[i];
      traj_act_msg.goal.seg_x[i].degree = order;

    }
    
    traj_goal_pub_.publish(traj_act_msg);
    std_srvs::Trigger trg;
    ros::service::call(poly_srv_name_, trg);

    return true;
  }


  void ReplanFSM::setYaw()
  {

    Eigen::Vector3d dir = end_pt_ - odom_pos_;
    kr_tracker_msgs::PolyTrackerActionGoal yaw_msg;
     
    desired_yaw_ = atan2(dir(1), dir(0));
    std::cout<< "desired_yaw_  is " << desired_yaw_  << std::endl;
    std::cout<< "odom_yaw_  is " << odom_yaw_  << std::endl;


    //yaw_msg.goal.frame_id = frame_id_;
    yaw_msg.goal.final_yaw = desired_yaw_;
    yaw_msg.goal.set_yaw = true;
    traj_goal_pub_.publish(yaw_msg);

    std_srvs::Trigger trg;
    ros::service::call(poly_srv_name_, trg);

  }

} // namespace 
