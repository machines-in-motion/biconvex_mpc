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
        ProblemData(int nx);

        // this sets the data for the optimization problem
        void set_data(Eigen::SparseMatrix<double> A, Eigen::VectorXd b, 
                      Eigen::VectorXd P_k, double rho);

        // function to set cost function
        void set_cost(Eigen::SparseMatrix<double> Q, Eigen::VectorXd q){
            Q_ = Q; q_ = q;
        }

        //Compute cost function for given x
        double compute_obj(Eigen::VectorXd x_k);
        double compute_obj_diff();

        //Compute gradient of cost function for a given x
        void compute_grad_obj();

        // warm starting x
        void set_warm_x(Eigen::VectorXd x_wm){x_k = x_wm;}

        double rho_;
        double obj_ = 0.0;

        Eigen::VectorXd lb_;
        Eigen::VectorXd ub_;
        Eigen::VectorXd P_k_;

        Eigen::SparseMatrix<double> Q_;
        Eigen::VectorXd q_;
        Eigen::SparseMatrix<double> ATA_;
        Eigen::SparseMatrix<double> A_;
        Eigen::VectorXd b_;
        Eigen::VectorXd bPk_;
        Eigen::VectorXd ATbPk_;


        //FISTA related optimization variables
        Eigen::VectorXd y_k;
        Eigen::VectorXd x_k;
        Eigen::VectorXd y_k_1;
        Eigen::VectorXd x_k_1;
        Eigen::VectorXd y_diff;
        Eigen::VectorXd gradient; 

        double prev_obj;
        double G_k_norm;
        double G_k_norm_inf_max = 0.0;
        double G_k_norm_inf_min = 0.0;
    };
} //namespace function

#endif