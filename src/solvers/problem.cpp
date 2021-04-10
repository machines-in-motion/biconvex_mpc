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
        //prev_obj = 0.0;
//        y_k = Eigen::VectorXd::Zero(n_);
//        y_k_1 = Eigen::VectorXd::Zero(n_);
//        x_k = Eigen::VectorXd::Zero(n_);
//        x_k_1 = Eigen::VectorXd::Zero(n_);
//        y_diff = Eigen::VectorXd::Zero(n_);
//        gradient = Eigen::VectorXd::Zero(n_);

        Q_sp = Q_e.sparseView();
        A_sp = A_e.sparseView();

        // auto t1 = high_resolution_clock::now();

        ATA_sp = 2*(Q_sp + rho_*(A_sp).transpose()*(A_sp));
        ATbPk_e = 2.0*rho_*(A_sp).transpose()*(-b_e + Pk_e) + q_e;
        bPk_ = -b_e + Pk_e;

        // ATbPk_sp = ATbPk_e.sparseView();
        // auto t2  = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = t2 - t1;
        //std::cout << "matrix creation " << ms_double.count() << "ms" << std::endl;
    }

    double ProblemData::compute_obj(const Eigen::VectorXd& x) {
        //auto t1 = high_resolution_clock::now();
        obj = (rho_)*((A_sp*x + bPk_).squaredNorm());
        for (unsigned i = 0 ; i < n_; ++i){
            obj += Q_e(i,i)*x(i)*x(i) + q_e(i)*x(i);
        }
        // auto t2  = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "obj " << ms_double.count() << "ms" << std::endl;
        // std::cout << obj << std::endl;
        return obj;
    }

    void ProblemData::compute_grad_obj(const Eigen::VectorXd& x) {
        // auto t1 = high_resolution_clock::now();
        gradient = ATA_sp*x + ATbPk_e;
        // auto t2  = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "grad_obj " <<  ms_double.count() << "ms" << std::endl;

        // x_2 = x.sparseView();
        // auto t1_sp = high_resolution_clock::now();
        // gradient2 = ATA_sp*x_2 + ATbPk_sp;
        // auto t2_sp  = high_resolution_clock::now();
        // duration<double, std::milli> ms_double_sp = t2_sp - t1_sp;
        // std::cout << "grad_obj with sparse matrices " <<  ms_double_sp.count() << "ms" << std::endl;
        // gradient = Eigen::VectorXd(gradient2);
        //return grad;
    }

} //namespace fista