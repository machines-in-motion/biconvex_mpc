#include "fista.hpp"

#include <iostream>

namespace solvers
{
    FISTA::FISTA(double L0, double beta, double tolerance) : l0_(L0), beta_(beta), tolerance_(tolerance){
        std::cout << "Setting up FISTA solver" << std::endl;
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
    
    void FISTA::optimize(std::shared_ptr<function::ProblemData> prob_data){
        auto t_k = 1.0;
        auto t_k_1 = t_k;
        for (int i=0; i<max_iters; ++i) {
            compute_step_length(prob_data);
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

} //namespace solvers