#include "problem.hpp"

#include <iostream>

namespace function
{
    ProblemData::ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q, 
                                std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b,
                                std::shared_ptr<Eigen::VectorXd> P_k, std::shared_ptr<int> n, 
                                std::shared_ptr<double> rho)
    : Q_(Q), q_(q), A_(A), b_(b), Pk_(Pk_), n(n), rho(rho){
        std::cout << "Setting up problem data" << std::endl;

        *ATA_ = *rho*(*A_).transpose()*(*A_);
        *ATbPk_ = (*A_).transpose()*(-*b_ + *Pk_) + *q_;
        *bPk_ = *rho*(-*b_ + *Pk_);
    }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        // auto obj =  x.transpose()*(*Q_)*x + (*q_).dot(x) + (*rho*((*A_)*x - *b_ + *Pk_).norm());
        auto obj =  x.transpose()*(*Q_)*x + (*q_).dot(x) + ((*rho)*(*A_)*x - *bPk_).norm();
    
        return obj;
    }

    Eigen::VectorXd ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        Eigen::VectorXd grad =  2*(*Q_)*x + (*q_) + 2*(*rho)*(*A_).transpose()*((*A_)*x - *b_ + *Pk_);
        return grad;
    }
} //namespace fista