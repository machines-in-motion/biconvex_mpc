// Containst the centroidal dynamics class
// Author : Avadesh Meduri & Paarth Shah
// Date : 15/04/2021


#ifndef CENTROIDAL_HPP
#define CENTROIDAL_HPP
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

namespace dynamics{

    class CentroidalDynamics{

        public:
            CentroidalDynamics(double m, int n_col, int n_eff);
        
            void compute_x_mat(Eigen::VectorXd &X);
            void compute_f_mat(Eigen::VectorXd &F);
            
            void update_x_init(Eigen::VectorXd &x_init){
                for(unsigned t = 0; t < 9; ++t){
                    A_f.coeffRef(9*n_col_+t, t) = 1.0;
                    b_f[9*n_col_+t] = x_init[t];
                }; 
            };

            void set_contact_arrays(Eigen::MatrixXd cnt_plan, double dt);

            //Update the binary contact array
            void update_contact_array();

            Eigen::SparseMatrix<double> A_x;
            Eigen::VectorXd b_x;
            Eigen::SparseMatrix<double> A_f;
            Eigen::VectorXd b_f;
            Eigen::VectorXd x_init_;
        
            // location of the contact point array used to create constraints and for calculating forces/amom
            // Dimension: n_col_ x n_eff x 3
            std::vector<Eigen::MatrixXd> r_;

            // contact array that is used to create the constraints (tells if end effector is in contact)
            // Dimension: n_col_ x n_eff
            Eigen::MatrixXd cnt_arr_;

            // location of contact point at time t
            Eigen::MatrixXd r_t;

            const double m_;
            int n_col_;
            const double n_eff_;

            Eigen::VectorXd dt_;

            //TODO: 
    };

}

#endif