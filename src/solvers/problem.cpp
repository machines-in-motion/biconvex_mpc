#include "problem.hpp"

#include <iostream>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace function
{
    ProblemData::ProblemData(int nx) {
        
        y_k.resize(nx); y_k.setZero();
        y_k_1.resize(nx); y_k_1.setZero();
        x_k.resize(nx); x_k.setZero();
        x_k_1.resize(nx); x_k_1.setZero();
        
        gradient.resize(nx); gradient.setZero();
        y_diff.resize(nx); y_diff.setZero();
        // Should we create memory for the sparse matrices as well?
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
        // for (unsigned i = 0 ; i < n_; ++i){
        //     obj_ += Q_(i,i)*x(i)*x(i) + q_(i)*x(i);
        // }
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