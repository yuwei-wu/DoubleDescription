#ifndef _POLY_PLAN_H_
#define _POLY_PLAN_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <traj_utils/traj_min_jerk.hpp>

#include <traj_utils/lbfgs.hpp>
#include <unordered_map>
#include <traj_utils/math.h>
#include <traj_utils/plan_container.hpp>

using std::vector;

namespace opt_planner
{

  class PolySolver{
    
  public: 

    bool minJerkTrajOpt(Eigen::MatrixXd &inner_pts, // (3, N + 1)
                        Eigen::VectorXd &allo_ts,
                        Eigen::MatrixXd &iS, 
                        Eigen::MatrixXd &fS,
                        ros::Time ego_start_time,
                        std::vector<Eigen::MatrixXd> &hPolys);

    Eigen::VectorXd allocateTime(const Eigen::MatrixXd &wayPs,
                          double vel,
                          double acc);
    
    static double objCallback(void *ptrObj,
                              const double *x,
                              double *grad,
                              const int n);

    inline void init(Eigen::VectorXd wts, 
                     Eigen::VectorXd bds){
      w_ = wts;
      // w_time_,  w_vel_,  w_acc_,  w_sta_obs_
      b_ =  bds;
      bb_ = 0.3 * bds;
      jerkOpt_.setWeights(w_, b_);

    }

    void getTraj(min_jerk::Trajectory &traj){traj = minJerkTraj_;}
    min_jerk::JerkOpt jerkOpt_;

  private:

    min_jerk::Trajectory minJerkTraj_;
    Eigen::MatrixXd innerP_, alloT_, initS_, finalS_;
    std::vector<Eigen::MatrixXd> SFCs_;
    int iter_num_, piece_num_, var_num_, ego_id_;
    Eigen::VectorXd w_, b_, bb_;

    /*** feasibility checking ***/
    enum CHECKER_TYPE
    {
      VEL_INFI = 1,
      ACC_INFI = 2,
      STA_INFI = 3,
      DYN_INFI = 4,
      FEASIBLE = 5
    };

    double t_coeff_ = 1.05;
    int isFeasibile();
    void setInit();
    void refineInit(int checker);
    void splitSegments(const std::vector<int> idxs,
                       double time_coeff);

              
    void locateViolateStb(std::vector<int> &idxs);

    ros::Time ego_start_time_;

    void locateViolateDyn(std::vector<int> &idxs);


    Eigen::MatrixXd refinePoly(Eigen::MatrixXd hPoly,
                               Eigen::VectorXd hCol);
    bool insidehPoly(Eigen::MatrixXd hPoly,
                     Eigen::Vector3d pt);

    template <typename EIGENVEC>
    void VT2RT(Eigen::VectorXd &RT, const EIGENVEC &VT);

    template <typename EIGENVEC>
    void RT2VT(const Eigen::VectorXd &RT, EIGENVEC &VT);

  public: 
    typedef std::unique_ptr<PolySolver> Ptr;



  };
  
} // namespace

#endif