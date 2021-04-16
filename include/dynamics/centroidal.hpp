// Containst the centroidal dynamics class
// Author : Avadesh Meduri
// Date : 15/04/2021


#ifndef CENTROIDAL_HPP
#define CENTROIDAL_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

namespace dynamics{

    class CentroidalDynamics{

        public:
            CentroidalDynamics(double m, double dt, double T, double n_eff):
                m_(m), dt_(dt), T_(T), n_eff_(n_eff), n_col_(int (T_/dt_))
                {
                    // setting up A_f, and b_f
                    A_f.resize(9*(n_col_+1), 9*(n_col_+1));
                    b_f.resize(9*(n_col_+1));
                    b_f.setZero();
                    for (unsigned t = 0; t < n_col_; ++t){
                        for (unsigned l = 0; l < 3; ++l){
                            // creating identities
                            A_f.coeffRef(9*t+l,9*t+l) = 1.0;
                            A_f.coeffRef(9*t+l,9*(t+1)+l) = -1.0;
                            A_f.coeffRef(9*t+l,9*t+(l+3)) = dt_;
                        }
                        for (unsigned l = 3; l < 9; ++l){
                            // creating identities
                            A_f.coeffRef(9*t+l,9*t+l) = 1.0;
                            A_f.coeffRef(9*t+l,9*(t+1)+l) = -1.0;
                        }
                    }
      
                };
            
            
            
            Eigen::SparseMatrix<double> A_x;
            Eigen::VectorXd b_x;
            Eigen::SparseMatrix<double> A_f;
            Eigen::VectorXd b_f;
            Eigen::VectorXd x_init_;
            // void set_contact_plan(Eigen::MatrixXd r, Eigen::MatrixXd cnt_plan);

            void compute_x_mat(Eigen::VectorXd &X);
            void compute_f_mat(Eigen::VectorXd &F);
            
            void update_x_init(Eigen::VectorXd x_init){
                for(unsigned t = 0; t < 9; ++t){
                    A_f.coeffRef(n_col_+t, t) = 1.0;
                    b_f[n_col_+t] = x_init[t];
                }; 
            };

        Eigen::MatrixXd cnt_plan_;
        std::vector<Eigen::VectorXd> r_;

        private:

            const double m_;
            const double dt_;
            const double T_;
            const double n_eff_;
            const int n_col_;
    };

}

#endif