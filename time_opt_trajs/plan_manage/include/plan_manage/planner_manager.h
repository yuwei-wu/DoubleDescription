#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <local_mapping/grid_map.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include <plan_manage/poly_opt.h>
#include <traj_utils/planning_visualization.h>
#include <path_searching/kinodynamic_astar.h>

namespace opt_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class PlannerManager
  {
    // SECTION stable
  public:
    PlannerManager();
    ~PlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;

    //trajs data
    LocalTrajData local_data_;


    //map utils
    GridMap::Ptr grid_map_;
    EllipsoidDecomp3D decomp_util_;


    //@yuwei display settings 
    Eigen::Vector4d display_color_;

    //@yuwei local multi-agent clearance primitives
    std::unique_ptr<KinodynamicAstar> kino_path_finder_;
    double time_res_ = 0.2;
    bool kinoPlan(Eigen::MatrixXd &startState,
                  Eigen::MatrixXd &endState,
                  ros::Time plan_time,
                  std::vector<Eigen::Vector3d> &kino_path);


    bool getSikangConst(std::vector<Eigen::Vector3d> &path_pts,
                        Eigen::MatrixXd &inner_pts,
                        Eigen::VectorXd &allo_ts,
                        std::vector<Eigen::MatrixXd> &hPolys);

    void setGlobalGoal(Eigen::Vector3d &end_pt){
      global_end_pt_ = end_pt;
    }

    double  local_plan_horizon_;
    Eigen::Vector3d global_end_pt_;


    std::string frame_id_;

    ros::Publisher poly_pub_ ;

    // optimization
    PolySolver poly_traj_solver_;

    bool localPlanner(Eigen::MatrixXd &startState, 
                      Eigen::Vector3d &yawStartState);
    bool have_opt_path_ = false;

    double bb_back_ = 0.5;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    int continous_failures_count_{0};

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<PlannerManager> Ptr;

    // !SECTION
  };
} // namespace

#endif