#include "solvers/fista.hpp"


namespace solvers
{
    void FISTA::compute_step_length(function::ProblemData & prob_data_) {
        prob_data_.compute_grad_obj();    
        while (1) {
            prob_data_.y_k_1 = (prob_data_.y_k - prob_data_.gradient/L_).cwiseMin(prob_data_.ub_).cwiseMax(prob_data_.lb_);
            prob_data_.y_diff = (prob_data_.y_k_1 - prob_data_.y_k); // proximal gradient
            prob_data_.G_k_norm = prob_data_.y_diff.norm(); // proximal gradient norm
            if (prob_data_.compute_obj_diff() > prob_data_.gradient.transpose()*(prob_data_.y_diff) +
                                                                    (L_/2)*(prob_data_.G_k_norm*prob_data_.G_k_norm)){
                L_ = beta_*L_;
                // std::cout << "Line search called - " << L_ << std::endl;

            }
            else {
                prob_data_.x_k_1 = prob_data_.y_k_1;
                break;
            }
        }
    }

    void FISTA::optimize(function::ProblemData & prob_data_, int max_iters, double tol){
        prob_data_.y_k = prob_data_.x_k;
        t_k = 1.0;
        // auto t1 = high_resolution_clock::now();
        for (int i=0; i<max_iters; ++i) {
            compute_step_length(prob_data_);
            // std::cout << "ls" <<  L_ << std::endl;
            t_k_1 = 1.0 + sqrt(1 + 4*t_k*t_k)/2.0;
            prob_data_.y_k_1 = prob_data_.x_k_1 + ((t_k-1)/t_k_1)*(prob_data_.x_k_1 - prob_data_.x_k);
            
            prob_data_.x_k = prob_data_.x_k_1;
            // prob_data_.x_k.swap(prob_data_.x_k_1);
            if(prob_data_.G_k_norm < tol) {
                // std::cout << "terminated due to norm. Iters " << i << std::endl;
                // auto t2  = high_resolution_clock::now();
                break;
            }

            // see if this step can be removed
            prob_data_.y_k = prob_data_.y_k_1;
            // prob_data_.y_k.swap(prob_data_.y_k_1);

            t_k = t_k_1;
        }
        // auto t2 = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "fista time" << ms_double.count() << "ms" << std::endl;
        // std::cout << "Terminating due to exit criteria ..." << max_iters_ << std::endl;
    }

} //namespace solvers