#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <chrono>
#include <memory>

namespace function
{
class ProblemData 
    {
    public:
        ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q,
                    std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b,
                    std::shared_ptr<Eigen::VectorXd> P_k, std::shared_ptr<int> n,
                    std::shared_ptr<double> rho);

        ProblemData(Eigen::MatrixXd Q, Eigen::VectorXd q,
                    Eigen::MatrixXd A, Eigen::VectorXd b, 
                    Eigen::VectorXd P_k, int n, double rho);

        //Compute cost function for given x
        double compute_obj(const Eigen::VectorXd& x);

        //Compute cost function for given x. Temp. functionality for pybind testing
        double compute_obj_pybind(const Eigen::VectorXd& x);

        //Compute gradient of cost function for a given x
        Eigen::VectorXd compute_grad_obj(const Eigen::VectorXd& x);

        //Compute gradient of objective function for a given x. Temp. functionality for pybind testing
        Eigen::VectorXd compute_grad_obj_pybind(const Eigen::VectorXd& x);

        void set_bounds(Eigen::VectorXd lb, Eigen::VectorXd ub) {lb_ = lb; ub_ = ub;}

        /**
         * The following data should be moved to private with getters and setters
         */
    
        Eigen::VectorXd lb_;
        Eigen::VectorXd ub_;
        double rho_e;
        int n_e;
        //Temporary PyBind variables to get around Eigen::Ref issues
        Eigen::MatrixXd Q_e;
        Eigen::VectorXd q_e;
        Eigen::MatrixXd A_e;
        Eigen::VectorXd b_e;
        Eigen::VectorXd Pk_e;

        Eigen::SparseMatrix<double> Q_sp;
        // Eigen::VectorXd q_e;
        // Eigen::MatrixXd A_e;
        // Eigen::VectorXd b_e;
        // Eigen::VectorXd Pk_e;


        Eigen::MatrixXd ATA_;
        Eigen::VectorXd Pk_;
        Eigen::VectorXd bPk_;
        Eigen::VectorXd ATbPk_;


        //FISTA related optimization variables
        Eigen::VectorXd y_k;
        Eigen::VectorXd y_k_1;
        Eigen::VectorXd x_k;
        Eigen::VectorXd x_k_1;
        double G_k_norm;
    };
} //namespace function

#endif