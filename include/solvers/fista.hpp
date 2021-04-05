#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "problem.hpp"

namespace solvers
{
class FISTA
    {
    public:
        FISTA(double L0, double beta, double tolerance = 0.00001);

        //Optimize function
        void optimize(std::shared_ptr<function::ProblemData> prob_data);

        //Compute Step Length
        void compute_step_length(std::shared_ptr<function::ProblemData> problem);

        //Resets parameters
        void reset();

        //Set beta
        void set_beta(double beta) { beta_ = beta; }
        //Set l0
        void set_l0(double l0) { l0_ = l0; }
        //Set tolerance
        void set_tolerance(double tolerance) { tolerance_ = tolerance; }

    private:
        //Computes step length
        void compute_step_length(Eigen::VectorXd y_k);

        //Solver parameters
        double l0_;
        double tolerance_;
        double beta_;
        int max_iters;
    };
} //namespace solvers

#endif