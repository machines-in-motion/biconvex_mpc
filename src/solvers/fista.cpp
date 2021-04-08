#include "fista.hpp"

#include <iostream>

namespace solvers
{
    FISTA::FISTA(double L0, double beta, int max_iters, double tolerance):
    l0_(L0), beta_(beta), max_iters_(max_iters), tolerance_(tolerance){
        std::cout << "Setting up FISTA solver with settings: " << std::endl;
        std::cout << "l0: " << l0_ << " beta:" << beta_ << " max. iterations: " << max_iters_ <<
        " tolerance:" << tolerance_ << std::endl;
    }

    void FISTA::compute_step_length(std::shared_ptr<function::ProblemData> problem) {
        auto L = l0_;
        auto grad_k = problem->compute_grad_obj((problem->y_k));

        // This should probably have a stopping criteria just in case...
        while (1) {
            problem->y_k_1 = ((problem->y_k) - grad_k/L).cwiseMin(problem->lb_).cwiseMax(problem->ub_);
            problem->G_k_norm = (problem->y_k_1 - problem->y_k).norm(); // proximal gradient
            if (problem->compute_obj(problem->y_k_1) > 
                problem->compute_obj(problem->y_k) + (grad_k.transpose())*(problem->y_k_1 - problem->y_k) + 
                                                                                        (L/2)*(problem->G_k_norm)){
                    L = beta_*L;
                }
            else {
                break;
            }
        }
    }

    void FISTA::compute_step_length_pybind() {
        auto L = l0_;
        auto grad_k = prob_data_.compute_grad_obj(prob_data_.y_k_1);

        // This should probably have a stopping criteria just in case...
        while (1) {
            prob_data_.y_k_1 = ((prob_data_.y_k) - grad_k/L).cwiseMin(prob_data_.lb_).cwiseMax(prob_data_.ub_);
            prob_data_.G_k_norm = (prob_data_.y_k_1 - prob_data_.y_k).norm(); // proximal gradient
            if (prob_data_.compute_obj(prob_data_.y_k_1) >
                prob_data_.compute_obj(prob_data_.y_k) + (grad_k.transpose())*(prob_data_.y_k_1 - prob_data_.y_k) +
                (L/2)*(prob_data_.G_k_norm)){
                L = beta_*L;
            }
            else {
                break;
            }
        }
    }
    
    void FISTA::optimize(std::shared_ptr<function::ProblemData> prob_data){
        auto t_k = 1.0;
        auto t_k_1 = t_k;
        for (int i=0; i<max_iters_; ++i) {
            compute_step_length(prob_data); //Temporarily commented out to test pybind functionality
            t_k_1 = 1.0 + sqrt(1 + 4*std::pow(t_k,2))/2.0;
            prob_data->y_k_1 = prob_data->x_k_1 + ((t_k-1)/t_k_1)*(prob_data->x_k_1 - prob_data->x_k);
            
            prob_data->y_k = prob_data->y_k_1;
            t_k = t_k_1;

            if(prob_data->G_k_norm < tolerance_){
                std::cout << "Terminating due to exit criteria ..." << std::endl;
                break;
            }
        }
    }

    void FISTA::optimize_pybind(){
        auto t_k = 1.0;
        auto t_k_1 = t_k;
        for (int i=0; i<max_iters_; ++i) {
            //compute_step_length(prob_data); //Temporarily commented out to test pybind functionality
            compute_step_length_pybind();
            t_k_1 = 1.0 + sqrt(1 + 4*std::pow(t_k,2))/2.0;
            prob_data_.y_k_1 = prob_data_.x_k_1 + ((t_k-1)/t_k_1)*(prob_data_.x_k_1 - prob_data_.x_k);
            prob_data_.y_k = prob_data_.y_k_1;
            t_k = t_k_1;

            if(prob_data_.G_k_norm < tolerance_){
                std::cout << "Terminating due to exit criteria ..." << std::endl;
                break;
            }
        }
    }

    void FISTA::set_data(Eigen::MatrixXd Q, Eigen::VectorXd q, Eigen::MatrixXd A, Eigen::VectorXd b,
                         Eigen::VectorXd P_k,Eigen::VectorXd ub, Eigen::VectorXd lb, double rho, int n) {
        // prob_data_.Q_e = Q;
        // prob_data_.q_e = q;
        // prob_data_.A_e = A;
        // prob_data_.b_e = b;

        // prob_data_.Pk_e = b;

        prob_data_(Q, q, A, b, P_k, n, rho);

        std::cout << "Q: " << std::endl << prob_data_.Q_e << std::endl;
        std::cout << "b: " << std::endl << prob_data_.b_e << std::endl;

        prob_data_.y_k = Eigen::VectorXd(b.size());
        prob_data_.y_k_1 = Eigen::VectorXd(b.size());
        prob_data_.x_k = Eigen::VectorXd(b.size());
        prob_data_.x_k_1 = Eigen::VectorXd(b.size());

        std::cout << "Size of y_k (and other FISTA variables): " << prob_data_.y_k.size() << std::endl;

        prob_data_.rho_ = rho;
        prob_data_.n_ = n;
        prob_data_.set_bounds(ub, lb);
    }
} //namespace solvers