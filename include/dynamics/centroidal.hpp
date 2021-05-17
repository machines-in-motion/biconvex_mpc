// Containst the centroidal dynamics class
// Author : Avadesh Meduri
// Date : 15/04/2021


#ifndef CENTROIDAL_HPP
#define CENTROIDAL_HPP
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

namespace dynamics{

    class CentroidalDynamics{

        public:
            CentroidalDynamics(double m, double dt, double T, int n_eff);
        
            void compute_x_mat(Eigen::VectorXd &X);
            void compute_f_mat(Eigen::VectorXd &F);
            
            void update_x_init(Eigen::VectorXd &x_init){
                for(unsigned t = 0; t < 9; ++t){
                    A_f.coeffRef(9*n_col_+t, t) = 1.0;
                    b_f[9*n_col_+t] = x_init[t];
                }; 
            };

            void create_contact_array();

            // contact plan provided by the user (# x [1/0, x, y, z, start, end])
            std::vector<Eigen::MatrixXd> cnt_plan_;

            Eigen::SparseMatrix<double> A_x;
            Eigen::VectorXd b_x;
            Eigen::SparseMatrix<double> A_f;
            Eigen::VectorXd b_f;
            Eigen::VectorXd x_init_;
        
        // private:
            // location of the contact point array used to create constraints
            std::vector<Eigen::MatrixXd> r_;
            // contact array that is used to create the constraints (tells if end effecto is in contact)
            Eigen::MatrixXd cnt_arr_; // = Eigen::MatrixXd::Zero(n_col_, n_eff_);
            // This array is used to create the cnt_arr_, r_arr_
            Eigen::VectorXd t_arr;
            double time_steps;
            // location of contact point at time t
            Eigen::MatrixXd r_t;

            const double m_;
            const double dt_;
            const double T_;
            const double n_eff_;

            //TODO: 
            //Change to horizon_ or knots_ (or something else)
            const int n_col_;
    };

}

#endif