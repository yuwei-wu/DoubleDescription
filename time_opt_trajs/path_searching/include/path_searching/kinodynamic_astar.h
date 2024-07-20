#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

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


namespace opt_planner {

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
public:
  /* -------------------- */
  Eigen::Vector3i index;  // sample jerk
  Eigen::Matrix<double, 9, 1> state;  // pos, vel, acc

  
  /* yaw dynamics primitves*/ 
  
  Eigen::Vector2d yaw_state;
  double yaw_input; // ddd yaw
  int yaw_idx;


  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;

  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  } 
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;

template <class T>
class NodeComparator {
public:
  bool operator()(T node1, T node2) {
    return node1->f_score > node2->f_score;
  }
};



template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


template <class T>
class NodeHashTable {
private:
  /* data */
  typedef Eigen::Matrix<int, 5, 1> Vector5i;

  std::unordered_map<Eigen::Vector3i, T, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, T, matrix_hash<Eigen::Vector4i>> data_4d_; // with yaw or time
  std::unordered_map<Vector5i, T, matrix_hash<Vector5i>> data_5d_; // with yaw and time

public:
  NodeHashTable(/* args */) {
  }
  ~NodeHashTable() {
  }
  void insert(Eigen::Vector3i idx, T node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int yt_idx, T node) {
    data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), yt_idx), node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, int yaw_idx, T node) {
    data_5d_.insert(std::make_pair(Vector5i(idx(0), idx(1), idx(2), time_idx, yaw_idx), node));
  }


  T find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  T find(Eigen::Vector3i idx, int yt_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), yt_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  T find(Eigen::Vector3i idx, int time_idx, int yaw_idx) {
    auto iter = data_5d_.find(Vector5i(idx(0), idx(1), idx(2), time_idx, yaw_idx));
    return iter == data_5d_.end() ? NULL : iter->second;
  }


  void clear() {
    data_3d_.clear();
    data_4d_.clear();
    data_5d_.clear();
  }
};


class KinodynamicAstar {
private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;

  int use_node_num_, iter_num_;

  NodeHashTable<PathNodePtr> expanded_nodes_; // the candidate nodes 
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator<PathNodePtr>> open_set_;
  std::vector<PathNodePtr> path_nodes_;
  /* ---------- record data ---------- */
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, end_acc_;
  Eigen::Matrix<double, 9, 9> phi_;  // state transit matrix

  GridMap::Ptr grid_map_;

  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.5;
  double max_vel_  = 3.0;
  double max_acc_  = 3.0;
  double max_jerk_ = 2.0;
  double max_dyaw_  = 0.8;
  double max_ddyaw_ = 0.2;

  double horizon_ = 7.5;
  double lambda_heu_;
  double half_fov_theta_ = 2.0;

  int vis_check_num_ = 10;

  double w_time_;

  //double margin_;
  int allocate_num_;
  double resolution_, time_resolution_;

  /* map */
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

  
  inline double range(double angle){
    // range the angle into (-PI, PI]
    double psi = angle;

    if (angle > M_PI)
    {
      psi = 2 * M_PI - angle;
    }
    else if (angle <= -M_PI)
    {
      psi = angle + 2 * M_PI;
    }
    return psi;
  }

  /* checking related*/
  double ego_radius_;
  int ego_id_;

  /* perception value computation*/

public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};

  /* main API */
  void setParam(ros::NodeHandle &nh);
  void init(int ego_id, double ego_radius);
  void reset();
  int search(Eigen::MatrixXd startState, 
             Eigen::MatrixXd endState, 
             ros::Time time_start, 
             bool init, bool dynamic = false);

  void intialMap(GridMap::Ptr &grid_map);
  void getKinoTraj(double delta_t, std::vector<Eigen::Vector3d> &path_list);
                     
  typedef shared_ptr<KinodynamicAstar> Ptr;
  
};

}  // namespace 

#endif
