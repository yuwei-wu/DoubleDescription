#ifndef _KINO_ACC_ASTAR_H
#define _KINO_ACC_ASTAR_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>
#include "local_mapping/grid_map.h"
#include <traj_utils/math.h>
#include <path_searching/kino_utils.hpp>

namespace opt_planner {

class KinoAccAstar {
private:
  /* ---------- main data structure ---------- */
  vector<AccNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable<AccNodePtr> expanded_nodes_; // the candidate nodes 
  std::priority_queue<AccNodePtr, std::vector<AccNodePtr>, NodeComparator<AccNodePtr>> open_set_;
  std::vector<AccNodePtr> path_nodes_;
  /* ---------- record data ---------- */
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, end_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  Eigen::MatrixXd coef_shot_;
  vector<Eigen::Vector3d> acc_inputs_;
  double t_shot_;
  bool is_shot_succ_ = false;
  bool has_path_ = false;
  KinoSearchParameters ksp_;

  /* map */
  GridMap::Ptr grid_map_;
  /* shot trajectory */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);

public:
  KinoAccAstar(){};
  ~KinoAccAstar();

  /* main API */
  void setParam(ros::NodeHandle &nh);
  void init();
  void reset();
  void intialMap(GridMap::Ptr &grid_map);
  void getKinoTraj(double delta_t, std::vector<Eigen::Vector3d> &path_list);
  int search(Eigen::MatrixXd startState, 
             Eigen::MatrixXd endState, 
             ros::Time time_start, 
             bool init);
                     
  typedef shared_ptr<KinoAccAstar> Ptr;
  
};

}  // namespace 

#endif
