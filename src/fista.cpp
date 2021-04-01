#include "fista.hpp"

#include <iostream>

namespace fista
{
    Solver::Solver(double L0, double beta, double tolerance = 0.00001) : l0_(L0), beta_(beta), tolerance_(tolerance){
        std::cout << "Setting up FISTA solver" << std::endl;
    }

    void Solver::optimize(std::shared_ptr<ProblemData> prob_data){
        auto t_k = 1.0;
        auto t_k_1 = t_k;
        for (int i=0; i<max_iters; ++i) {
            compute_step_length(prob_data);
            t_k_1 = 1.0 + sqrt(1 + 4*std::pow(t_k,2))/2.0;
            problem->y_k_1 = x_k_1 + ((t_k-1)/t_k_1)*(x_k_1 - x_k);
        }
    }

    void Solver::compute_step_length(std::shared_ptr<ProblemData> problem) {
        auto L = l0_;
        auto grad_k = problem->compute_grad_obj(*(problem->y_k));

        // This should probably have a stopping criteria just in case...
        while (1) {
            (problem->y_k_1) = ((problem->y_k) - grad_k/L).min(problem->getLowerBound()).max(problem->getUpperBound());
            (problem->G_k_norm) = (problem->y_k_1) - problem->y_k).normSquared(); // proximal gradient
            if (problem->compute_obj(problem->y_k_1) > 
                problem->compute_obj(problem->y_k) + (grad_k.tranpsose())*(problem->y_k_1 - problem->y_k) + (L/2)*(problem->G_k_norm)){
                    L = beta_*L;
                }
            else {
                break;
            }
        }
    }
    
} //namespace fista