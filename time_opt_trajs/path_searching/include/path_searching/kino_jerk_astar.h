#ifndef _KINO_JERK_ASTAR_H
#define _KINO_JERK_ASTAR_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>
#include <traj_utils/math.h>
#include <path_searching/kino_utils.hpp>

#include <mpl_collision/map_util.h>


namespace opt_planner {

using namespace std;
class KinoJerkAstar {
private:
  /* ---------- main data structure ---------- */
  vector<JerkNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable<JerkNodePtr> expanded_nodes_; // the candidate nodes 
  std::priority_queue<JerkNodePtr, std::vector<JerkNodePtr>, NodeComparator<JerkNodePtr>> open_set_;
  std::vector<JerkNodePtr> path_nodes_;
  /* ---------- record data ---------- */
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, end_acc_;
  Eigen::Matrix<double, 9, 9> phi_;  // state transit matrix
  bool is_shot_succ_ = false;
  bool has_path_ = false;
  Eigen::MatrixXd coef_shot_;
  vector<Eigen::Vector3d> jerk_inputs_;
  double t_shot_;
  KinoSearchParameters ksp_;

  /* map */
  std::shared_ptr<MPL::VoxelMapUtil> kino_map_util_;

  /* shot trajectory */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 9, 1>& state0, Eigen::Matrix<double, 9, 1>& state1,
                    Eigen::Vector3d um, double tau);
  /* perception value computation*/

public:
  KinoJerkAstar(){};
  ~KinoJerkAstar();

  /* main API */
  void setParam(ros::NodeHandle &nh);
  void init();
  void reset();
  void intialMap(const std::shared_ptr<MPL::VoxelMapUtil> &map_util);
  void getKinoTraj(double delta_t, std::vector<Eigen::Vector3d> &path_list);
  int search(Eigen::MatrixXd startState, 
             Eigen::MatrixXd endState, 
             ros::Time time_start, 
             bool init);

  typedef shared_ptr<KinoJerkAstar> Ptr;
  
};

}  // namespace 

#endif
