#ifndef _KINO_JERK_ASTAR_H
#define _KINO_JERK_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <math.h>
#include "local_mapping/grid_map.h"
#include <utility>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/root_finder.hpp>
#include <traj_utils/traj_min_jerk.hpp>
#include <path_searching/kino_utils.hpp>



namespace opt_planner {


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
  double t_shot_;

  /* map */
  GridMap::Ptr grid_map_;
  double time_origin_;
  Eigen::Vector3d origin_;

  /* helper */ 
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);

  template <typename T>
  void retrievePath(T end_node, std::vector<T> &nodes);

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
  void init(int ego_id, double ego_radius);
  void reset();
  void intialMap(GridMap::Ptr &grid_map);
  void getKinoTraj(double delta_t, std::vector<Eigen::Vector3d> &path_list);
  int search(Eigen::MatrixXd startState, 
             Eigen::MatrixXd endState, 
             ros::Time time_start, 
             bool init, bool dynamic = false);

  typedef shared_ptr<KinoJerkAstar> Ptr;
  
};

}  // namespace 

#endif
