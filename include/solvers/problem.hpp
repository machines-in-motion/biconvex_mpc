#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <eigen3/Eigen/Dense>
#include <memory>

namespace function
{
/**
 *  Class storing the data to solve the problem (0.5 x*Q*x + 2*qx) s.t. Ax = b
*/
class ProblemData 
{
public:
    ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q, 
                std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b);

    //Compute cost function for given x
    double compute_obj(const Eigen::VectorXd& x);

    //Compute gradient of cost function for a given x
    Eigen::VectorXd compute_grad_obj(const Eigen::VectorXd& x);

    void set_bounds(Eigen::VectorXd lb, Eigen::VectorXd ub) {lb_ = lb; ub_ = ub;}

    /**
     * The following data should be moved to private with getters and setters
     */
    std::shared_ptr<Eigen::MatrixXd> Q_;
    std::shared_ptr<Eigen::VectorXd> q_;
    std::shared_ptr<Eigen::MatrixXd> A_;
    std::shared_ptr<Eigen::VectorXd> b_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
    std::shared_ptr<Eigen::VectorXd> Pk_;
    double rho;

    //FISTA related optimization variables
    Eigen::VectorXd y_k;
    Eigen::VectorXd y_k_1;
    Eigen::VectorXd x_k;
    Eigen::VectorXd x_k_1;
    double G_k_norm;

private:
};

} //namespace function

#endif