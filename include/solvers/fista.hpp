#ifndef SOLVERS_HPP
#define SOLVERS_HPP
#include <iostream>
#include "problem.hpp"
#include <thread>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


namespace solvers
{
class FISTA
    {
    public:
    
        FISTA(){};

        //Optimize function
        void optimize(function::ProblemData & prob_data_, int max_iters, double tol);

        //Compute Step Length
        void compute_step_length(function::ProblemData & prob_data_);

        //Resets parameters
        void reset();

        //Set beta
        void set_beta(double beta) { beta_ = beta; }
        //Set l0
        void set_l0(double l0) { L_ = l0; }

        //Use Second Order Cone Projection
        void set_soc_true() { use_soc_projection_ = true; }

        //Set Friction Cone
        void set_friction_coefficient(double mu) { mu_ = mu; }

    private:
        //Computes step length
        void compute_step_length(Eigen::VectorXd y_k);

        //Computes second order cone projection for constraints
        void SoC_projection(function::ProblemData & prob_data_);

        //KEEP THIS FALSE
        bool use_soc_projection_ = false;
        
        //Solver parameters
        double L_ = 150;
        double beta_ = 1.5;

        double soc_norm = 0;

        double t_k;
        double t_k_1;

        double mu_ = 1.0;
    };
} //namespace solvers

#endif