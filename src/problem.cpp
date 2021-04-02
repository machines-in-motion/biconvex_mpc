#include "problem.hpp"

#include <iostream>

namespace function
{
    ProblemData::ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q, std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b)
    : Q_(Q), q_(q), A_(A), b_(b){
        std::cout << "Setting up problem data" << std::endl;
    }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        auto obj =  x.transpose()*(*Q_)*x + (*q_).dot(x) + (rho*((*A_)*x - *b_ + *Pk_).norm());
        return obj;
    }

    Eigen::VectorXd ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        Eigen::VectorXd grad =  2*(*Q_)*x + (*q_) + 2*rho*(*A_).transpose()*((*A_)*x - *b_ + *Pk_);
        return grad;
    }
    
} //namespace fista