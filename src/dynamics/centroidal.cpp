#include "dynamics/centroidal.hpp"


namespace dynamics{

    CentroidalDynamics::CentroidalDynamics(double m, int n_col, int n_eff):
                m_(m), n_col_(n_col), n_eff_(n_eff)
        {
            dt_.resize(n_col_); dt_.setZero();
            // setting up A_f, and b_f (For optimizing for CoM, Vel, AMOM)
            A_f.resize(9*(n_col_+1), 9*(n_col_+1));
            b_f.resize(9*(n_col_+1));
            b_f.setZero();
            for (unsigned t = 0; t < n_col_; ++t){
                for (unsigned l = 0; l < 3; ++l){
                    // creating identities
                    A_f.coeffRef(9*t+l,9*t+l) = 1.0;
                    A_f.coeffRef(9*t+l,9*(t+1)+l) = -1.0;
                }
                for (unsigned l = 3; l < 9; ++l){
                    // creating identities
                    A_f.coeffRef(9*t+l,9*t+l) = 1.0;
                    A_f.coeffRef(9*t+l,9*(t+1)+l) = -1.0;
                }
            }

            // setting up A_x, b_x (For optimizing for forces and torques)
            A_x.resize(9*(n_col_+1), 3*n_eff_*n_col_);
            b_x.resize(9*(n_col_+1));
            b_x.setZero();

            cnt_arr_.resize(n_col_, n_eff_);
            cnt_arr_.setZero();

            r_t.resize(n_eff_, 3);
            r_t.setZero();
    };

    void CentroidalDynamics::set_contact_arrays(Eigen::MatrixXd cnt_plan, double dt){
        r_.push_back(r_t);
        int i = r_.size() -1 ;
        for (unsigned j = 0; j < n_eff_; ++j) {
            dt_[i] = dt;
            cnt_arr_(i, j) = cnt_plan(j, 0);
            r_[i](j, 0) = cnt_plan(j, 1);
            r_[i](j, 1) = cnt_plan(j, 2);
            r_[i](j, 2) = cnt_plan(j, 3);
        }
    };

    void CentroidalDynamics::update_contact_array(){
        for (unsigned int i = 0; i < n_col_; ++i) {
            cnt_arr_.row(i) = cnt_arr_.row(i+1);
        }
    }

    void CentroidalDynamics::compute_x_mat(Eigen::VectorXd &X){

        for (unsigned t = 0; t < n_col_; ++t){
            b_x[9*t+3] = X[9*(t+1)+3] - X[9*t+3];
            b_x[9*t+4] = X[9*(t+1)+4] - X[9*t+4];
            b_x[9*t+5] = X[9*(t+1)+5] - X[9*t+5] + 9.81*dt_[t];
            b_x[9*t+6] = (X[9*(t+1)+6] - X[9*t+6]); 
            b_x[9*t+7] = (X[9*(t+1)+7] - X[9*t+7]);
            b_x[9*t+8] = (X[9*(t+1)+8] - X[9*t+8]);

            for (unsigned n = 0; n < n_eff_; ++n){
                // velocity constraints
                A_x.coeffRef(9*t+3, 3*n_eff_*t + 3*n) = cnt_arr_(t,n)*(dt_[t]); // normalized forces
                A_x.coeffRef(9*t+4, 3*n_eff_*t + 3*n + 1) = cnt_arr_(t,n)*(dt_[t]);
                A_x.coeffRef(9*t+5, 3*n_eff_*t + 3*n + 2) = cnt_arr_(t,n)*(dt_[t]);

                // AMOM constraints
                A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+1) = cnt_arr_(t,n)*(X((9*t)+2) - r_[t](n,2))*dt_[t];
                A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+2) = -cnt_arr_(t,n)*(X((9*t)+1) - r_[t](n,1))*dt_[t];

                A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+0) = -cnt_arr_(t,n)*(X((9*t)+2) - r_[t](n,2))*dt_[t];
                A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+2) = cnt_arr_(t,n)*(X((9*t)+0) - r_[t](n,0))*dt_[t];

                A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+0) = cnt_arr_(t,n)*(X((9*t)+1) - r_[t](n,1))*dt_[t];
                A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+1) = -cnt_arr_(t,n)*(X((9*t)+0) - r_[t](n,0))*dt_[t];
            }
        }
    };

    void CentroidalDynamics::compute_f_mat(Eigen::VectorXd &F){

        // auto F = F1*m_; // de normalizing the F vector
        for (unsigned t = 0; t < n_col_; ++t){
            A_f.coeffRef(9*t+0,9*(t+1)+(0+3)) = dt_[t];
            A_f.coeffRef(9*t+1,9*(t+1)+(1+3)) = dt_[t];
            A_f.coeffRef(9*t+2,9*(t+1)+(2+3)) = dt_[t];

            A_f.coeffRef(9*t+6, 9*t+1) = -cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_[t];
            A_f.coeffRef(9*t+6, 9*t+2) = cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_[t];
            
            A_f.coeffRef(9*t+7, 9*t+0) = cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_[t];
            A_f.coeffRef(9*t+7, 9*t+2) = -cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_[t];
            
            A_f.coeffRef(9*t+8, 9*t+0) = -cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_[t];
            A_f.coeffRef(9*t+8, 9*t+1) = cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_[t];
            
            b_f[9*t+3] = -cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_[t];
            b_f[9*t+4] = -cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_[t];
            b_f[9*t+5] = -cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_[t] + 9.81*dt_[t];
            b_f[9*t+6] = (cnt_arr_(t,0)*F[3*t*n_eff_+1]*r_[t](0,2) - cnt_arr_(t,0)*F[3*t*n_eff_+2]*r_[t](0,1))*dt_[t];
            b_f[9*t+7] = (cnt_arr_(t,0)*F[3*t*n_eff_+2]*r_[t](0,0) - cnt_arr_(t,0)*F[3*t*n_eff_+0]*r_[t](0,2))*dt_[t];
            b_f[9*t+8] = (cnt_arr_(t,0)*F[3*t*n_eff_+0]*r_[t](0,1) - cnt_arr_(t,0)*F[3*t*n_eff_+1]*r_[t](0,0))*dt_[t];
            
            for (unsigned n = 1; n < n_eff_; ++n){
                A_f.coeffRef(9*t+6, 9*t+1) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*dt_[t];
                A_f.coeffRef(9*t+6, 9*t+2) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*dt_[t];
                
                A_f.coeffRef(9*t+7, 9*t+0) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*dt_[t];
                A_f.coeffRef(9*t+7, 9*t+2) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*dt_[t];
                
                A_f.coeffRef(9*t+8, 9*t+0) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*dt_[t];
                A_f.coeffRef(9*t+8, 9*t+1) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*dt_[t];
                
                b_f[9*t+3] += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*dt_[t];
                b_f[9*t+4] += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*dt_[t];
                b_f[9*t+5] += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*dt_[t];
                b_f[9*t+6] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*r_[t](n,2) - cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t](n,1))*dt_[t];
                b_f[9*t+7] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t](n,0) - cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t](n,2))*dt_[t];
                b_f[9*t+8] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t](n,1) - cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*r_[t](n,0))*dt_[t];
            }
        }
    };

}