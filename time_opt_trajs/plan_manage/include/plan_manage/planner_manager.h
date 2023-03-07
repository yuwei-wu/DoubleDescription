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
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include <plan_manage/poly_opt.h>
#include <traj_utils/planning_visualization.h>
#include <path_searching/kino_acc_astar.h>
#include <path_searching/kino_jerk_astar.h>

#include "sdlp.hpp"
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
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    void initPlanModules(ros::NodeHandle &nh, 
                         const std::shared_ptr<MPL::VoxelMapUtil> &map_util);

    PlanParameters pp_;

    //trajs data
    LocalTrajData local_data_;


    //map utils
    std::shared_ptr<MPL::VoxelMapUtil> map_util_;
    EllipsoidDecomp3D decomp_util_;

    //@yuwei local multi-agent clearance primitives
    std::unique_ptr<KinoJerkAstar> kinojerk_path_finder_;
    std::unique_ptr<KinoAccAstar> kinoacc_path_finder_;

    double time_res_ = 0.1;
    template <typename T>
    bool kinoPlan(Eigen::MatrixXd &startState,
                  Eigen::MatrixXd &endState,
                  ros::Time plan_time,
                  std::vector<Eigen::Vector3d> &kino_path,
                  T &finder);


    bool getSikangConst(std::vector<Eigen::Vector3d> &path_pts,
                        Eigen::MatrixXd &inner_pts,
                        Eigen::VectorXd &allo_ts,
                        std::vector<Eigen::MatrixXd> &hPolys);


    std::string frame_id_;

    ros::Publisher poly_pub_ ;

    // optimization
    PolySolver poly_traj_solver_;

    bool localPlanner(Eigen::MatrixXd &startState, Eigen::MatrixXd &endState);
    bool have_opt_path_ = false;
    bool use_jerk_ = false;

    void setTraj(std::vector<double> &dura,
                std::vector<min_jerk::BoundaryCond> &bCond);


    double bb_xy_size_, bb_z_up_, bb_z_down_;

  private:
    //Yifei added debugging
    bool if_write_poly_file_ = true;
    uint local_planner_iteration_ = 0;
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;


    //poly is defined as h0*x + h1*y + h2*z + h3 <= 0 
    inline bool overlap(const Eigen::MatrixXd &hPoly0,
                        const Eigen::MatrixXd &hPoly1,
                        const double eps = 1.0e-6)

    {

        unsigned int m = hPoly0.cols();
        unsigned int n = hPoly1.cols();

        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);

        Eigen::MatrixX3d normals0 = (hPoly0.bottomRows<3>()).transpose();  //  (m, 3)
        Eigen::MatrixX3d normals1 = (hPoly1.bottomRows<3>()).transpose();  //  (n, 3)

        A.leftCols<3>().topRows(m) = normals0; // m * 3
        A.leftCols<3>().bottomRows(n) = normals1;
        A.rightCols<1>().setConstant(1.0);

        for (int i = 0; i < m; i ++)
        {
           b(i) =  normals0.row(i).dot(hPoly0.col(i).head<3>());
        }
        for (int j = 0; j < n; j ++)
        {
           b(m+j) =  normals1.row(j).dot(hPoly1.col(j).head<3>());
        }

        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

  void visPoly(std::vector<Eigen::MatrixXd> &hPolys)
  {
    decomp_ros_msgs::PolyhedronArray poly_msg;
    for (const auto &hPoly : hPolys)
    {
      decomp_ros_msgs::Polyhedron msg;
      geometry_msgs::Point pt, n;
        
      for (unsigned int i = 0; i < hPoly.cols(); i++)
      {  
        pt.x = hPoly(0, i);
        pt.y = hPoly(1, i);
        pt.z = hPoly(2, i);
        n.x  = hPoly(3, i);
        n.y  = hPoly(4, i);
        n.z  = hPoly(5, i);
        msg.points.push_back(pt);
        msg.normals.push_back(n);
      }
      poly_msg.polyhedrons.push_back(msg);
    }

    poly_msg.header.frame_id = frame_id_;
    poly_pub_.publish(poly_msg);

    return;
  }



  public:
    typedef unique_ptr<PlannerManager> Ptr;

    // !SECTION
  };
} // namespace

#endif