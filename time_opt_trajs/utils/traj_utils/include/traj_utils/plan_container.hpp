#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <traj_utils/traj_min_jerk.hpp>


using std::vector;

namespace opt_planner
{

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_, max_dyaw_, max_ddyaw_; // physical limits

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;

    /* agent information */
    int mav_id_;
    double  mav_radius_;
    
    /* multi-agent cases */
    double multi_clearance_range_;

  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    min_jerk::Trajectory traj_;
  };


  struct OneTrajDataOfSwarm
  {
    /* info of generated traj */
    int mav_id_;
    double mav_radius_;
    int traj_id_;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    min_jerk::Trajectory traj_;
  };

  typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData;
  

} // namespace

#endif