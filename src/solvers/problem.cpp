#include "solvers/problem.hpp"

#include <iostream>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace function
{
    ProblemData::ProblemData(int state, int horizon) {
        state_ = state;
        horizon_ = horizon;
        num_vars_ = state_*horizon;

        y_k.resize(num_vars_); y_k.setZero();
        y_k_1.resize(num_vars_); y_k_1.setZero();
        x_k.resize(num_vars_); x_k.setZero();
        x_k_1.resize(num_vars_); x_k_1.setZero();
        
        gradient.resize(num_vars_); gradient.setZero();
        y_diff.resize(num_vars_); y_diff.setZero();

        lb_.resize(num_vars_); lb_.setZero();
        ub_.resize(num_vars_); ub_.setZero();

        Q_.resize(num_vars_, num_vars_); q_.resize(num_vars_);
        q_.setZero();
    }

    void ProblemData::set_data(Eigen::SparseMatrix<double> A, Eigen::VectorXd b, 
                            Eigen::VectorXd P_k, double rho){

        A_ = A; b_ = b; P_k_ = P_k; rho_ = rho;

        ATA_ = 2*(Q_ + rho_*(A_).transpose()*(A_));
        bPk_ = -b_ + P_k_;
        ATbPk_ = 2.0*rho_*(A_).transpose()*(bPk_) + q_; 
        }

    double ProblemData::compute_obj(Eigen::VectorXd x_k) {
        obj_ = x_k.transpose()*Q_*x_k + q_.dot(x_k) + (rho_)*((A_*x_k + bPk_).squaredNorm());
        return obj_;
    }

    double ProblemData::compute_obj_diff() {
        obj_ = (y_k_1 + y_k).transpose()*Q_*(y_k_1 - y_k) + q_.dot(y_k_1 - y_k) + 
                    (rho_)*(((A_*y_k_1 + bPk_).squaredNorm()) - ((A_*y_k + bPk_).squaredNorm()));

        return obj_;
    }


    void ProblemData::compute_grad_obj() {
        gradient = ATA_*y_k + ATbPk_;
    }

} //namespace fista