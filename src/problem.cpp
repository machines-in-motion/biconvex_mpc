#include "problem.hpp"

#include <iostream>

namespace function
{
    ProblemData::ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q, std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b)
    : Q_(Q), q_(q), A_(A), b_(b){
        std::cout << "Setting up problem data" << std::endl;
    }

    ProblemData::compute_obj(const Eigen::VectorXd& x) {
        return x.transpose()*(*Q_)*x + (*q_).transpose()*x + (rho*((*A_)*x - b_ + Pk_).norm());
    }

    ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        return 2*(*Q_)*x + (*q_) + 2*rho*(*A_).transpose()*((*A_)*x - b_ + Pk_);
    }
    
} //namespace fista