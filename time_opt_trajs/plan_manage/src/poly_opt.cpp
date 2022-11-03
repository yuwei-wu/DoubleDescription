// #include <fstream>
#include <plan_manage/poly_opt.h>
#include <thread>

namespace opt_planner
{

  bool PolySolver::minJerkTrajOpt(Eigen::MatrixXd &inner_pts, //(3, N -1)
                                  Eigen::VectorXd &allo_ts,
                                  Eigen::MatrixXd &iS,
                                  Eigen::MatrixXd &fS,
                                  ros::Time ego_start_time,
                                  std::vector<Eigen::MatrixXd> &hPolys)
  {

    innerP_ = inner_pts;
    alloT_ = allo_ts;
    SFCs_ = hPolys;
    initS_ = iS;
    finalS_ = fS;
    ego_start_time_ = ego_start_time;

    setInit();
    // ROS_WARN("PolySolver::minJerkTrajOpt");
    // ROS_INFO_STREAM("innerP " << innerP_);
    // ROS_INFO_STREAM("alloT " << alloT_);
    // ROS_INFO_STREAM("THE initS_  " << initS_);
    // ROS_INFO_STREAM("THE finalS_ " << finalS_);

    std::chrono::high_resolution_clock::time_point tc1, tc2;
    double d1 = 0.0;
    tc1 = std::chrono::high_resolution_clock::now();
    ros::Time t0 = ros::Time::now(), t1, t2;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 60;
    lbfgs_params.mem_size = 32;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-2;

    /* ---------- prepare ---------- */
    double final_cost = 0;
    iter_num_ = 0;
    bool is_success = false, is_feasible = false;
    int infeasible_num = 0, fail_num = 0;


    /* ---------- optimize ---------- */
    do
    {

      double init_vars[var_num_];
      memcpy(init_vars, innerP_.data(), innerP_.size() * sizeof(init_vars[0]));
      //memcpy(init_vars + innerP_.size(), alloT.data(), alloT.size()* sizeof(init_vars[0]));
      Eigen::Map<Eigen::VectorXd> VT(init_vars + innerP_.size(), alloT_.size());
      RT2VT(alloT_, VT);

      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(var_num_,
                                         init_vars,
                                         &final_cost,
                                         PolySolver::objCallback,
                                         nullptr,
                                         nullptr,
                                         this,
                                         &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;

      jerkOpt_.getTraj(minJerkTraj_);
      int checker = isFeasibile();

      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {

        if (checker == CHECKER_TYPE::FEASIBLE && !isnan(final_cost))
        {
          is_success = true;

          printf("\033[32mSuccess: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);

          tc2 = std::chrono::high_resolution_clock::now();
          d1 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
          std::cout << "Piece Number: MinJerk Comp. Time: " << d1 << " s" << std::endl;
        }
        else
        {
          // insert new points
          printf("\033[33mInfeasible: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
          infeasible_num++;
          refineInit(checker);
          setInit();
        }
      }
      else
      {

        if (checker == CHECKER_TYPE::FEASIBLE)
        {
          printf("\033[32mFeasible solution: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
          is_feasible = true; // compromise to a feasible solution
          break;
        }
        refineInit(checker);
        setInit();
        printf("\033[34mFails: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
        fail_num++;
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }
    //std::cout << "+++++++++++++++++++++++++++++++  " << std::endl;
    iter_num_ = 0;
    } while (is_success == false && infeasible_num < 3 && fail_num < 3);

    std::cout << "[minJerk Optimizer]Total Time is " << (t2 - t0).toSec() * 1000 << " ms" << std::endl;
    // 1. success and no infeasible points    is_success == true
    // 2. fails but has feasible solution     is_feasible == true
    // 3. fails upper 3
    return (is_success || is_feasible);
  }


  template <typename EIGENVEC>
  void PolySolver::VT2RT(Eigen::VectorXd &RT, const EIGENVEC &VT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC>
  void PolySolver::RT2VT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  void PolySolver::setInit()
  {

    piece_num_ = alloT_.size();
    var_num_ = 4 * (piece_num_ - 1) + 1;

    // set the optimizer
    jerkOpt_.reset(initS_, finalS_, piece_num_);
    jerkOpt_.generate(innerP_, alloT_);
  }

  void PolySolver::refineInit(int checker)
  {

    // refine the trajectory
    //std::cout << "[PolySolver::refineInit]" << std::endl;
    std::vector<int> idxs;

    switch (checker)
    {

      case CHECKER_TYPE::VEL_INFI:
      {

        minJerkTraj_.locateMaxVelRate(b_(0) + bb_(0), idxs);
        splitSegments(idxs, t_coeff_);

        break;
      }
      case CHECKER_TYPE::ACC_INFI:
      {
        minJerkTraj_.locateMaxAccRate(b_(1) + bb_(1), idxs);
        splitSegments(idxs, t_coeff_);

        break;
      }
      case CHECKER_TYPE::STA_INFI:
      {

        locateViolateStb(idxs);
        splitSegments(idxs, 1.0);
        w_(2) *= t_coeff_;
        jerkOpt_.setWeights(w_, 0.9 * b_);

        // double check

        break;
      }

    }

    return;
  }

  void PolySolver::locateViolateStb(std::vector<int> &idxs)
  {

    idxs.clear();
    Eigen::VectorXd durs = minJerkTraj_.getDurations();
    double a, b, c, d, e, f;
    for (size_t i = 0; i < piece_num_; i++)
    { // for each segmentv
      int corr_k = SFCs_[i].cols();
      //  TrajDim, TrajOrder + 1
      min_jerk::CoefficientMat coeff = minJerkTraj_[i].getCoeffMat(); // no t
      bool is_collide = false;
      for (int k = 0; k < corr_k && !is_collide; k++)
      {

        Eigen::Vector3d point = SFCs_[i].col(k).head<3>();
        Eigen::Vector3d outer_n = SFCs_[i].col(k).tail<3>();

        a = outer_n.dot(coeff.col(0));
        b = outer_n.dot(coeff.col(1));
        c = outer_n.dot(coeff.col(2));
        d = outer_n.dot(coeff.col(3));
        e = outer_n.dot(coeff.col(4));
        f = outer_n.dot(coeff.col(5) - point);

        std::vector<double> ts = solve(a, b, c, d, e, f);

        // relative time
        for (const auto &it : ts)
        {
          if (it >= 0 && it <= durs[i])
          {
            idxs.push_back(i);
            //std::cout << "collide! the index is : i" << i << " time is " << it << std::endl;
            Eigen::Vector3d pt = minJerkTraj_[i].getPos(it);
            //std::cout <<"[debug] module 2 , insidehPoly(SFCs_[i], pt)  "  << insidehPoly(SFCs_[i], pt) << std::endl;
            //std::cout <<"[debug] module 2 , insidehPoly(SFCs_[i], minJerkTraj_[i].getPos(it+0.1))  "  << insidehPoly(SFCs_[i], minJerkTraj_[i].getPos(it+0.1)) << std::endl;
            is_collide = true;
            break;
          }
        }
      }
    }

    return;
  }

  void PolySolver::splitSegments(const std::vector<int> idxs,
                                 double time_coeff)
  {

    int add_num = idxs.size();
    Eigen::VectorXd newT(piece_num_ + add_num);
    Eigen::MatrixXd newP(3, piece_num_ - 1 + add_num), hPoly1, hPoly2;
    std::vector<Eigen::MatrixXd> newSFCs;
    Eigen::VectorXd hCol_left(6), hCol_right(6);

    size_t j = 0; // 0 - idx.size()-1
    double cur_time = 0;
    //std::cout <<"[debug] module , the idx size is" <<  add_num << "  the piece_num_is " << piece_num_ << std::endl;

    Eigen::MatrixXd total_pts = minJerkTraj_.getPositions();
    Eigen::VectorXd total_durs = minJerkTraj_.getDurations();

    //std::cout <<"[debug] module , total_pts is" <<  total_pts << std::endl;
    for (size_t i = 0; i < piece_num_; i++)
    {
      // have the insert point
      if (j < add_num && i == idxs.at(j))
      {

        //std::cout <<"[debug] i is " <<  i  << "  j is " << j << " idxs.at(j) " <<  idxs.at(j) << std::endl;
        newT(i + j) = time_coeff * 0.5 * total_durs(i);
        newT(i + j + 1) = newT(i + j);

        // the middle point is the line between the two point
        newP.col(i + j) = 0.5 * (total_pts.col(i) + total_pts.col(i + 1));

        if (i < piece_num_ - 1)
        {
          newP.col(i + j + 1) = total_pts.col(i + 1);
        }

        hCol_left.head(3) = 0.4 * total_pts.col(i) + 0.6 * total_pts.col(i + 1);
        hCol_right.head(3) = 0.6 * total_pts.col(i) + 0.4 * total_pts.col(i + 1);
        hCol_left.tail(3) = (total_pts.col(i + 1) - total_pts.col(i)).normalized(); // the normal must be outside
        hCol_right.tail(3) = -hCol_left.tail(3);

        hPoly1 = refinePoly(SFCs_.at(i), hCol_left);
        hPoly2 = refinePoly(SFCs_.at(i), hCol_right);
        newSFCs.push_back(hPoly1);
        newSFCs.push_back(hPoly2);


        j++;
      }
      else
      {
        newT(i + j) = total_durs(i);

        if (i < piece_num_ - 1)
        {
          newP.col(i + j) = total_pts.col(i + 1);
        }
        newSFCs.push_back(SFCs_.at(i));
      }

      cur_time += total_durs(i);
    }

    innerP_ = newP;
    alloT_ = newT;
    SFCs_ = newSFCs;
    // ROS_INFO_STREAM("innerP " << innerP_);
    // ROS_INFO_STREAM("alloT " << alloT_);
  }

  Eigen::MatrixXd PolySolver::refinePoly(Eigen::MatrixXd hPoly, //6 * n
                                         Eigen::VectorXd hCol)
  {

    unsigned int corr_k = hPoly.cols();
    Eigen::MatrixXd A(3, 3), new_hPoly(6, corr_k + 1);
    Eigen::Vector3d n1, p1, n2, p2, n3, pt, b;
    new_hPoly.setZero();

    int idx = 0;
    p2 = hCol.head(3);
    n2 = hCol.tail(3);

    //std::cout <<"[debug]  insidehPoly(hPoly, p2) " <<  insidehPoly(hPoly, p2) << std::endl;
    // std::cout <<"[debug]  hPoly" <<  hPoly << std::endl;
    // std::cout <<"[debug]  hCol" <<  hCol << std::endl;
    for (int k = 0; k < corr_k; k++)
    {
      n1 = hPoly.col(k).tail<3>();
      if (n1.dot(n2) > 1e-6)
      {
        p1 = hPoly.col(k).head<3>();
        n3 = n1.cross(n2);

        A.row(0) = n1.transpose();
        A.row(1) = n2.transpose();
        A.row(2) = n3.transpose();

        pt = A.inverse() * Eigen::Vector3d(n1.dot(p1), n2.dot(p2), n3.dot(p2));

        //std::cout <<"[debug] refine poly ,A.inverse() is " << A.inverse() << std::endl;

        if (insidehPoly(hPoly, pt))
        {

          //std::cout <<"[debug] refine poly ,cut plane, the point is " << pt << std::endl;

          new_hPoly.col(idx) = hPoly.col(k);
          idx++;
        }
      }
      else
      {
        new_hPoly.col(idx) = hPoly.col(k);
        idx++;
      }
    }

    new_hPoly.col(idx) = hCol;
    //std::cout <<"[debug]  new_hPoly is " <<  new_hPoly << std::endl;
    //std::cout <<"[debug]  new_hPoly.leftCols(idx+1)  is " <<  new_hPoly.leftCols(idx+1) << std::endl;
    //std::cout <<"[debug]  insidehPoly(new_hPoly.leftCols(idx+1), p2) " <<  insidehPoly(new_hPoly.leftCols(idx+1), p2) << std::endl;
    return new_hPoly.leftCols(idx + 1);
  }

  bool PolySolver::insidehPoly(Eigen::MatrixXd hPoly,
                               Eigen::Vector3d pt)
  {

    unsigned int corr_num = hPoly.cols();
    Eigen::Vector3d p_, n_;
    for (int i = 0; i < corr_num; i++)
    {

      p_ = hPoly.col(i).head<3>();
      n_ = hPoly.col(i).tail<3>();

      if (n_.dot(pt - p_) > 1e-5)
      {
        return false;
      }
    }
    return true;
  }

  // return true if the trajectory is satisfying
  int PolySolver::isFeasibile()
  {

    // check the dynamic limits
    if (!minJerkTraj_.checkMaxVelRate(b_(0) + bb_(0)))
    {
      //std::cout << " minJerkTraj_.getMaxVelRate() " << minJerkTraj_.getMaxVelRate() << std::endl;
      std::cout << " [PolySolver::Status] Velocity infeasible ... "<< std::endl;
      return CHECKER_TYPE::VEL_INFI;
    }
    if (!minJerkTraj_.checkMaxAccRate(b_(1) + bb_(1)))
    {
      //std::cout << " minJerkTraj_.getMaxAccRate() " << minJerkTraj_.getMaxAccRate() << std::endl;
      std::cout << " [PolySolver::Status] Acceleration infeasible ... "<< std::endl;
      return CHECKER_TYPE::ACC_INFI;
    }

    // check the static collision free
    Eigen::VectorXd durs = minJerkTraj_.getDurations();
    double a, b, c, d, e, f;
    for (unsigned int i = 0; i < piece_num_; i++)
    { // for each segmentv
      int corr_k = SFCs_[i].cols();
      //  TrajDim, TrajOrder + 1
      min_jerk::CoefficientMat coeff = minJerkTraj_[i].getCoeffMat();
      for (int k = 0; k < corr_k; k++)
      {

        Eigen::Vector3d outer_n = SFCs_[i].col(k).tail<3>();
        Eigen::Vector3d point = SFCs_[i].col(k).head<3>();

        a = outer_n.dot(coeff.col(0));
        b = outer_n.dot(coeff.col(1));
        c = outer_n.dot(coeff.col(2));
        d = outer_n.dot(coeff.col(3));
        e = outer_n.dot(coeff.col(4));
        f = outer_n.dot(coeff.col(5) - point);

        std::vector<double> ts = solve(a, b, c, d, e, f);

        // relative time
        for (const auto &it : ts)
        {
          if (it >= 0 && it <= durs[i])
          {
            // Eigen::Vector3d test_p = coeff.col(0) * std::pow(it, 5) +
            //                          coeff.col(1) * std::pow(it, 4) +
            //                          coeff.col(2) * std::pow(it, 3) +
            //                          coeff.col(3) * std::pow(it, 2) +
            //                          coeff.col(4) * std::pow(it, 1) +
            //                          coeff.col(5) * std::pow(it, 0);

            Eigen::Vector3d pt = minJerkTraj_[i].getPos(it);
            //std::cout <<"[debug] module 2 , test_p "  << test_p << "   the pt is " << pt << std::endl;
            //std::cout <<"[debug] module 2 , insidehPoly(SFCs_[i], pt)  "  << insidehPoly(SFCs_[i], pt) << std::endl;
            std::cout << " [PolySolver::Status] SFC infeasible ... "<< std::endl;
            return CHECKER_TYPE::STA_INFI;
          }
        }
      }
    }


    return CHECKER_TYPE::FEASIBLE;
  }

  double PolySolver::objCallback(void *ptrObj,
                                 const double *x,
                                 double *grad,
                                 const int n)
  {
    PolySolver &obj = *(PolySolver *)ptrObj;
    
    //intialization
    // map for x
    Eigen::Map<const Eigen::MatrixXd> innerP(x, 3, obj.piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> alloVT(x + (3 * (obj.piece_num_ - 1)), obj.piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, obj.piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradVT(grad + (3 * (obj.piece_num_ - 1)), obj.piece_num_);
    gradVT.setZero();
    gradP.setZero();
    
    // convert the time T(>0) to e^T (virtual time)
    Eigen::VectorXd alloRT(obj.piece_num_);
    Eigen::VectorXd gradRT(obj.piece_num_);
    gradRT.setZero();
    obj.VT2RT(alloRT, alloVT);
    
    // map for gradients
    double smooth_cost = 0, piece_cost = 0;

    obj.jerkOpt_.generate(innerP, alloRT);

    smooth_cost = obj.jerkOpt_.getObjective(); //only the j hat
    gradRT += obj.jerkOpt_.getGradT();
    gradP += obj.jerkOpt_.getGradInnerP();

    // std::cout << "[PolySolver::objCallback] innerP " << innerP << std::endl;
    // std::cout << "[PolySolver::objCallback] alloT " << alloRT << std::endl;
    // std::cout << "[PolySolver::objCallback] gradP " << gradP << std::endl;
    //std::cout << "[PolySolver::objCallback] smooth_cost is " << smooth_cost << std::endl;
    // std::cout << "[PolySolver::objCallback] gradRT " << gradRT << std::endl;
    obj.jerkOpt_.addPieceCostGrad(gradRT, gradP, piece_cost, obj.SFCs_);
    //std::cout << "[PolySolver::objCallback] cost is " << piece_cost << std::endl;
    // std::cout << "[PolySolver::objCallback] gradT " << gradRT << std::endl;
    // std::cout << "[PolySolver::objCallback] gradP " << gradP << std::endl;

    obj.jerkOpt_.VTGrad(alloRT, alloVT, gradRT, gradVT);

    obj.iter_num_ += 1;

    return smooth_cost + piece_cost;
  }



} // namespace
