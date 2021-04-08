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
        ProblemData();

        ProblemData(std::shared_ptr<Eigen::MatrixXd> Q, std::shared_ptr<Eigen::VectorXd> q,
                    std::shared_ptr<Eigen::MatrixXd> A, std::shared_ptr<Eigen::VectorXd> b,
                    std::shared_ptr<Eigen::VectorXd> P_k, int n, double rho);

        ProblemData(Eigen::MatrixXd Q, Eigen::VectorXd q,
                    Eigen::MatrixXd A, Eigen::VectorXd b, 
                    Eigen::VectorXd P_k, int n, double rho);

        //Compute cost function for given x
        double compute_obj(const Eigen::VectorXd& x);
        double compute_obj_2(const Eigen::VectorXd& x);

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
        std::shared_ptr<Eigen::MatrixXd> Q_;
        std::shared_ptr<Eigen::VectorXd> q_;
        std::shared_ptr<Eigen::MatrixXd> A_;
        std::shared_ptr<Eigen::MatrixXd> ATA_;
        std::shared_ptr<Eigen::VectorXd> b_;
        std::shared_ptr<Eigen::VectorXd> Pk_;
        // std::shared_ptr<Eigen::VectorXd> bPk_;

        std::shared_ptr<Eigen::VectorXd> ATbPk_;

        double rho_;
        int n_;
        //Temporary PyBind variables to get around Eigen::Ref issues
        Eigen::VectorXd lb_;
        Eigen::VectorXd ub_;
        Eigen::MatrixXd Q_e;
        Eigen::VectorXd q_e;
        Eigen::MatrixXd A_e;
        Eigen::VectorXd b_e;
        Eigen::VectorXd Pk_e;
        Eigen::VectorXd bPk_;


        Eigen::SparseMatrix<double> Q_sp;
        // Eigen::VectorXd q_e;
        Eigen::SparseMatrix<double> A_sp;
        // Eigen::VectorXd b_e;
        // Eigen::VectorXd Pk_e;

        //FISTA related optimization variables
        Eigen::VectorXd y_k;
        Eigen::VectorXd y_k_1;
        Eigen::VectorXd x_k;
        Eigen::VectorXd x_k_1;
        double G_k_norm;
    };
} //namespace function

#endif