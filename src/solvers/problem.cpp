#include "problem.hpp"

#include <iostream>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace function
{
    ProblemData::ProblemData() {
        std::cout << "Made empty instance of Problem Data Class" << std::endl;
    }

    ProblemData::ProblemData(Eigen::MatrixXd Q, Eigen::VectorXd q,
                             Eigen::MatrixXd A, Eigen::VectorXd b, 
                             Eigen::VectorXd P_k, int n, double rho)
            : Q_e(Q), q_e(q), A_e(A), b_e(b), Pk_e(P_k), n_(n), rho_(rho){
        std::cout << "Setting up problem data with copied data" << std::endl;
        Q_sp.resize(Q_e.rows(), Q_e.cols());        
        for (unsigned i = 0 ; i < Q_e.rows(); i ++){
            Q_sp.insert(i,i) = Q_e(i,i);
        }
        Q_sp.makeCompressed();

        A_sp.resize(A_e.rows(), A_e.cols());        
        for (unsigned i = 0 ; i < A_e.rows(); i ++){
            for (unsigned j = 0 ; j < A_e.cols(); j ++){
                if (A_e(i,j) != 0){
                    A_sp.insert(i,j) = A_e(i,j);
                }
            }
        }
        A_sp.makeCompressed();

        auto t1 = high_resolution_clock::now();

        ATA_sp = 2*(Q_sp + rho_*(A_sp).transpose()*(A_sp));
        ATbPk_e = rho_*(A_sp).transpose()*(-b_e + Pk_e) + q_e;
        bPk_ = -b_e + Pk_e;

        auto t2  = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms" << std::endl;
    }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        
        auto t1 = high_resolution_clock::now();
        obj = (rho_)*(((A_sp)*x - b_e + Pk_e).norm());
        for (unsigned i = 0 ; i < n_; i ++){
            obj += Q_e(i,i)*x(i)*x(i) + q_e(i)*x(i);
        }
        auto t2  = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms" << std::endl;
        
        return obj;
    }

    Eigen::VectorXd ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        auto t1 = high_resolution_clock::now();
        Eigen::VectorXd grad =  ATA_sp*x + ATbPk_e;
        auto t2  = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms" << std::endl;
        return grad;
    }

    Eigen::VectorXd ProblemData::compute_grad_obj_pybind(const Eigen::VectorXd& x) {
        Eigen::VectorXd grad =  2*(Q_e)*x + (q_e) + 2*(rho_)*(A_e).transpose()*((A_e)*x - b_e + *Pk_);
        return grad;
    }

} //namespace fista