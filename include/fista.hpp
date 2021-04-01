#ifndef FISTA_HPP
#define FISTA_HPP

#include <Eigen/Dense>

#include "problem.hpp"

namespace fista
{
class Solver
    {
    public:
        Solver(double L0, double beta, double tolerance = 0.00001);

        //Optimize function
        void optimize(Eigen::VectorXd& f, Eigen::VectorXd& g, int maxit, double tol);

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

        //Use min and max Eigen functionality
        /*
        template <typename T>
        void clamp_vec(std::vector<T>& vec, const T& min, const T& max) {
            std::transform(std::begin(vec), std::end(vec), std::begin(vec),
                        [&] (const T& v) { return std::clamp(v, min, max); });
        };
        */

        //Solver parameters
        double l0_;
        double tolerance_;
        double beta_;
        int max_iters;
    };
} //namespace fista

#endif