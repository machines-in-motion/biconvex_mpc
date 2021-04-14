#include "fista.hpp"

#include <iostream>
#include <thread>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace solvers
{
    FISTA::FISTA(double L0, double beta, int max_iters, double tolerance):
    l0_(L0), beta_(beta), max_iters_(max_iters), tolerance_(tolerance){}

    void FISTA::compute_step_length() {
        prob_data_.gradient = prob_data_.compute_grad_obj(prob_data_.y_k);
        while (1) {
            prob_data_.y_k_1 = (prob_data_.y_k - prob_data_.gradient/L).cwiseMin(prob_data_.ub_).cwiseMax(prob_data_.lb_);
            prob_data_.y_diff = (prob_data_.y_k_1 - prob_data_.y_k); // proximal gradient
            prob_data_.G_k_norm = prob_data_.y_diff.norm(); // proximal gradient norm
            if (prob_data_.compute_obj(prob_data_.y_k_1) > 
                prob_data_.compute_obj(prob_data_.y_k) + prob_data_.gradient.transpose()*(prob_data_.y_diff) +
                                                                    (L/2)*(prob_data_.G_k_norm*prob_data_.G_k_norm)){
                    L = beta_*L;

                }
            else {
                prob_data_.x_k_1 = prob_data_.y_k_1;
                break;
            }
        }
    }

    Eigen::VectorXd FISTA::optimize(const Eigen::VectorXd& x){
        prob_data_.x_k = x;
        prob_data_.y_k = prob_data_.x_k;
        t_k = 1.0;
        L = l0_;
        auto t1 = high_resolution_clock::now();
        for (int i=0; i<max_iters_; ++i) {
            compute_step_length();
            // std::cout << "ls" <<  L << std::endl;
            t_k_1 = 1.0 + sqrt(1 + 4*t_k*t_k)/2.0;
            prob_data_.y_k_1 = prob_data_.x_k_1 + ((t_k-1)/t_k_1)*(prob_data_.x_k_1 - prob_data_.x_k);

            if(prob_data_.G_k_norm < tolerance_) {
                auto t2  = high_resolution_clock::now();
                break;
            }

            prob_data_.x_k = prob_data_.x_k_1;
            // prob_data_.x_k.swap(prob_data_.x_k_1);
            prob_data_.y_k = prob_data_.y_k_1;
            // prob_data_.y_k.swap(prob_data_.y_k_1);

            t_k = t_k_1;
        }
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << "fista time" << ms_double.count() << "ms" << std::endl;
        // std::cout << "Terminating due to exit criteria ..." << max_iters_ << std::endl;
        
        return prob_data_.x_k_1;
    }

    void FISTA::set_data(Eigen::MatrixXd Q, Eigen::VectorXd q, Eigen::MatrixXd A, Eigen::VectorXd b,
                         Eigen::VectorXd P_k,Eigen::VectorXd ub, Eigen::VectorXd lb, double rho, int n) {

        auto t1 = high_resolution_clock::now();
        max_iters_ = n;
        prob_data_ = function::ProblemData(Q, q, A, b, P_k, Q.cols(), rho);
        prob_data_.set_bounds(ub, lb);
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << "set data time" << ms_double.count() << "ms" << std::endl;
    }
} //namespace solvers