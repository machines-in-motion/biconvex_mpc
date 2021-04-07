#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "problem.hpp"

namespace solvers
{
class FISTA
    {
    public:
        FISTA(double L0, double beta, int max_iters, double tolerance = 0.00001);

        //Optimize function
        void optimize(std::shared_ptr<function::ProblemData> prob_data);

        //Optimize function
        void optimize_pybind();

        //Compute Step Length
        void compute_step_length(std::shared_ptr<function::ProblemData> problem);

        //Compute Step Length
        void compute_step_length_pybind();

        //Set data in ProblemData
        void set_data(Eigen::MatrixXd Q, Eigen::VectorXd q, Eigen::MatrixXd A, Eigen::VectorXd b,
                      Eigen::VectorXd ub, Eigen::VectorXd lb, double rho, int n);

        //Resets parameters
        void reset();

        //Set beta
        void set_beta(double beta) { beta_ = beta; }
        //Set l0
        void set_l0(double l0) { l0_ = l0; }
        //Set tolerance
        void set_tolerance(double tolerance) { tolerance_ = tolerance; }

        //Problem data for optimization
        function::ProblemData prob_data_;
    private:
        //Computes step length
        void compute_step_length(Eigen::VectorXd y_k);

        //Solver parameters
        double l0_;
        double tolerance_;
        double beta_;
        int max_iters_;
    };
} //namespace solvers

#endif