#include "problem.hpp"

#include <iostream>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace function
{
    ProblemData::ProblemData() {
        // std::cout << "Made empty instance of Problem Data Class" << std::endl;
    }

    ProblemData::ProblemData(Eigen::MatrixXd Q, Eigen::VectorXd q,
                             Eigen::MatrixXd A, Eigen::VectorXd b, 
                             Eigen::VectorXd P_k, int n, double rho)
            : Q_e(Q), q_e(q), A_e(A), b_e(b), Pk_e(P_k), n_(n), rho_(rho){

        Q_sp = Q_e.sparseView();
        A_sp = A_e.sparseView();

        
        ATA_sp = 2*(Q_sp + rho_*(A_sp).transpose()*(A_sp));
        bPk_ = -b_e + Pk_e;
        ATbPk_e = 2.0*rho_*(A_sp).transpose()*(bPk_) + q_e; 
        }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        obj = (rho_)*((A_sp*x + bPk_).squaredNorm());
        for (unsigned i = 0 ; i < n_; ++i){
            obj += Q_e(i,i)*x(i)*x(i) + q_e(i)*x(i);
        }
        return obj;
    }

    Eigen::VectorXd ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {

        gradient = ATA_sp*x + ATbPk_e;

        return gradient;
    }

    Eigen::VectorXd ProblemData::proj(Eigen::VectorXd& grad) {

        proj_grad  = (grad).cwiseMin(ub_).cwiseMax(lb_);
        return proj_grad;
    }


} //namespace fista