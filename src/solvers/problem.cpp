#include "problem.hpp"

#include <iostream>

namespace function
{
    
    ProblemData::ProblemData(Eigen::MatrixXd Q, Eigen::VectorXd q,
                             Eigen::MatrixXd A, Eigen::VectorXd b, 
                             Eigen::VectorXd P_k, int n, double rho)
            : Q_e(Q), q_e(q), A_e(A), b_e(b), Pk_e(P_k), n_e(n), rho_e(n){
        std::cout << "Setting up problem data with copied data" << std::endl;
        Q_sp.resize(Q_e.rows(), Q_e.cols());        
        for (unsigned i = 0 ; i < Q_sq.rows())

        
        // ATA_ = rho*(A_e).transpose()*(A_e);
        // ATbPk_ = (A_e).transpose()*(-b_e + Pk_e) + q_e;
        bPk_ = (-b_e + Pk_e);
    }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        // auto obj =  x.transpose()*(*Q_)*x + (*q_).dot(x) + (*rho*((*A_)*x - *b_ + *Pk_).norm());
        using std::chrono::high_resolution_clock;
        using std::chrono::duration_cast;
        using std::chrono::duration;
        using std::chrono::milliseconds;

        auto t1 = high_resolution_clock::now();
        auto obj =  x.transpose()*(Q_e)*x + (q_e).dot(x) + (rho_e)*(A_e*x - bPk_).norm();

        // double obj = 0;
        // for (unsigned i = 0 ; i < n_e; i ++){
        //     obj += Q_e(i)*x(i)*x(i) + q_e(i)*x(i);
        // }
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms" << std::endl;
    
        return obj;
    }

    // double ProblemData::compute_obj_pybind(const Eigen::VectorXd& x) {
    //     auto obj =  x.transpose()*(Q_e)*x + (q_e).dot(x) + ((*rho)*((A_e)*x - b_e + *Pk_).norm());
    //     return obj;
    // }

    Eigen::VectorXd ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        Eigen::VectorXd grad =  2*(Q_e)*x + (q_e) + 2*(rho_e)*(A_e).transpose()*((A_e)*x - b_e + Pk_e);
        return grad;
    }

    // Eigen::VectorXd ProblemData::compute_grad_obj_pybind(const Eigen::VectorXd& x) {
    //     Eigen::VectorXd grad =  2*(Q_e)*x + (q_e) + 2*(rho_e)*(A_e).transpose()*((A_e)*x - b_e + Pk_e);
    //     return grad;
    // }

} //namespace fista