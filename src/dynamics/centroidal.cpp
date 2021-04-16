#include "centroidal.hpp"


namespace dynamics{

    void CentroidalDynamics::compute_x_mat(Eigen::VectorXd &X){

        for (unsigned t = 0; t < n_col_; ++t){
            b_x[9*t+3] = X[9*(t+1)+3] - X[9*t+3];
            b_x[9*t+4] = X[9*(t+1)+4] - X[9*t+4];
            b_x[9*t+5] = X[9*(t+1)+5] - X[9*t+5] + 9.81*dt_;
            b_x[9*t+6] = X[9*(t+1)+6] - X[9*t+6];
            b_x[9*t+7] = X[9*(t+1)+7] - X[9*t+7];
            b_x[9*t+8] = X[9*(t+1)+8] - X[9*t+8];

            for (unsigned n = 0; n < n_eff_; ++n){
                // velocity constraints
                A_x.coeffRef(9*t+3, 3*n_eff_*t + 3*n) = cnt_plan_(t,n)*(dt_/m_);
                A_x.coeffRef(9*t+4, 3*n_eff_*t + 3*n + 1) = cnt_plan_(t,n)*(dt_/m_);
                A_x.coeffRef(9*t+5, 3*n_eff_*t + 3*n + 2) = cnt_plan_(t,n)*(dt_/m_);

                // AMOM constraints
                A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+1) = cnt_plan_(t,n)*(X((9*t)+2) - r_[t][n,2])*dt_;
                A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+2) = -cnt_plan_(t,n)*(X((9*t)+1) - r_[t][n,1])*dt_;

                A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+0) =-cnt_plan_(t,n)*(X((9*t)+2) - r_[t][n,2])*dt_;
                A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+2) = cnt_plan_(t,n)*(X((9*t)+0) - r_[t][n,0])*dt_;
                
                A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+0) = cnt_plan_(t,n)*(X((9*t)+1) - r_[t][n,1])*dt_;
                A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+1) = cnt_plan_(t,n)*(X((9*t)+0) - r_[t][n,0])*dt_;

            }
        }
    };

    void CentroidalDynamics::compute_f_mat(Eigen::VectorXd &F){
        for (unsigned t = 0; t < n_col_; ++t){
            A_f.coeffRef(9*t+6, 9*t+1) = -cnt_plan_(t,0)*F[3*t*n_eff_+2]*dt_;
            A_f.coeffRef(9*t+6, 9*t+2) = cnt_plan_(t,0)*F[3*t*n_eff_+1]*dt_;
            
            A_f.coeffRef(9*t+7, 9*t+0) = cnt_plan_(t,0)*F[3*t*n_eff_+2]*dt_;
            A_f.coeffRef(9*t+7, 9*t+2) = -cnt_plan_(t,0)*F[3*t*n_eff_+0]*dt_;
            
            A_f.coeffRef(9*t+8, 9*t+0) = -cnt_plan_(t,0)*F[3*t*n_eff_+1]*dt_;
            A_f.coeffRef(9*t+8, 9*t+1) = cnt_plan_(t,0)*F[3*t*n_eff_+0]*dt_;
            
            b_f[9*t+3] = -cnt_plan_(t,0)*F[3*t*n_eff_+0]*dt_/m_;
            b_f[9*t+4] = -cnt_plan_(t,0)*F[3*t*n_eff_+1]*dt_/m_;
            b_f[9*t+5] = -cnt_plan_(t,0)*F[3*t*n_eff_+2]*dt_/m_ + 9.81*dt_;
            b_f[9*t+6] = (cnt_plan_(t,0)*F[3*t*n_eff_+1]*r_[t][0,2] - cnt_plan_(t,0)*F[3*t*n_eff_+2]*r_[t][0,1])*dt_;
            b_f[9*t+7] = (cnt_plan_(t,0)*F[3*t*n_eff_+2]*r_[t][0,0] - cnt_plan_(t,0)*F[3*t*n_eff_+0]*r_[t][0,2])*dt_;
            b_f[9*t+8] = (cnt_plan_(t,0)*F[3*t*n_eff_+0]*r_[t][0,1] - cnt_plan_(t,0)*F[3*t*n_eff_+1]*r_[t][0,0])*dt_;
            
            for (unsigned n = 1; n < n_eff_; ++n){
                A_f.coeffRef(9*t+6, 9*t+1) += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+2]*dt_;
                A_f.coeffRef(9*t+6, 9*t+2) += cnt_plan_(t,n)*F[3*t*n_eff_+3*n+1]*dt_;
                
                A_f.coeffRef(9*t+7, 9*t+0) += cnt_plan_(t,n)*F[3*t*n_eff_+3*n+2]*dt_;
                A_f.coeffRef(9*t+7, 9*t+2) += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+0]*dt_;
                
                A_f.coeffRef(9*t+8, 9*t+0) += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+1]*dt_;
                A_f.coeffRef(9*t+8, 9*t+1) += cnt_plan_(t,n)*F[3*t*n_eff_+3*n+0]*dt_;
                
                b_f[9*t+3] += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+0]*dt_/m_;
                b_f[9*t+4] += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+1]*dt_/m_;
                b_f[9*t+5] += -cnt_plan_(t,n)*F[3*t*n_eff_+3*n+2]*dt_/m_ + 9.81*dt_;
                b_f[9*t+6] += (cnt_plan_(t,n)*F[3*t*n_eff_+3*n+1]*r_[t][0,2] - cnt_plan_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t][0,1])*dt_;
                b_f[9*t+7] += (cnt_plan_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t][0,0] - cnt_plan_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t][0,2])*dt_;
                b_f[9*t+8] += (cnt_plan_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t][0,1] - cnt_plan_(t,n)*F[3*t*n_eff_+3*n+1]*r_[t][0,0])*dt_;
            }
        }
    };

}