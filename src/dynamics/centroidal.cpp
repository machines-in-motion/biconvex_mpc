#include "dynamics/centroidal.hpp"


namespace dynamics{

    CentroidalDynamics::CentroidalDynamics(double m, double dt, double T, int n_eff):
                m_(m), dt_(dt), T_(T), n_eff_(n_eff), n_col_(int (ceil(T_/dt_)))
        {
            // setting up A_f, and b_f (For optimizing for CoM, Vel, AMOM)
            //std::cout << n_col_ << std::endl;
            A_f.resize(9*(n_col_+1), 9*(n_col_+1));
            b_f.resize(9*(n_col_+1));
            b_f.setZero();
            for (unsigned t = 0; t < n_col_; ++t){
                for (unsigned l = 0; l < 3; ++l){
                    // creating identities
                    A_f.coeffRef(9*t+l,9*t+l) = 1.0;
                    A_f.coeffRef(9*t+l,9*(t+1)+l) = -1.0;
                    A_f.coeffRef(9*t+l,9*(t+1)+(l+3)) = dt_;
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

            t_arr.resize(n_eff_);
            t_arr.setZero();
            cnt_arr_.resize(n_col_, n_eff_);

            r_t.resize(n_eff_, 3);
            r_t.setZero();
            cnt_arr_.setZero();

            dt_adaptive_.resize(n_col_+1);
            dt_adaptive_.setZero();
    };

    void CentroidalDynamics::create_contact_array(){
        for (unsigned i = 0; i < cnt_plan_.size(); ++i){
            for (unsigned k = 0; k < n_eff_; ++k){
                time_steps = (cnt_plan_[i](k,5) - cnt_plan_[i](k,4))/dt_;
                for (unsigned l = 0; l < time_steps; ++l){
                    cnt_arr_(t_arr(k)+l,k) = cnt_plan_[i](k,0);
                    if (k == 0){
                        r_.push_back(r_t);
                    }
                    r_[t_arr(k)+l](k,0) = cnt_plan_[i](k,1);
                    r_[t_arr(k)+l](k,1) = cnt_plan_[i](k,2);
                    r_[t_arr(k)+l](k,2) = cnt_plan_[i](k,3);
                }
                t_arr[k] += time_steps;
            }
        }
    };

    void CentroidalDynamics::create_contact_array_2(){
        for (unsigned i = 0; i < cnt_plan_2_.size(); ++i) {
            r_.push_back(r_t);
            for (unsigned j = 0; j < n_eff_; ++j) {
                cnt_arr_(i, j) = cnt_plan_2_[i](j, 0);
                r_[i](j, 0) = cnt_plan_2_[i](j, 1);
                r_[i](j, 1) = cnt_plan_2_[i](j, 2);
                r_[i](j, 2) = cnt_plan_2_[i](j, 3);
            }
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
            b_x[9*t+5] = X[9*(t+1)+5] - X[9*t+5] + 9.81*dt_;
            b_x[9*t+6] = X[9*(t+1)+6] - X[9*t+6];
            b_x[9*t+7] = X[9*(t+1)+7] - X[9*t+7];
            b_x[9*t+8] = X[9*(t+1)+8] - X[9*t+8];

            for (unsigned n = 0; n < n_eff_; ++n){
                if (cnt_arr_(t,n)) {
                    // velocity constraints
                    A_x.coeffRef(9*t+3, 3*n_eff_*t + 3*n) = (dt_/m_);
                    A_x.coeffRef(9*t+4, 3*n_eff_*t + 3*n + 1) = (dt_/m_);
                    A_x.coeffRef(9*t+5, 3*n_eff_*t + 3*n + 2) = (dt_/m_);

                    // AMOM constraints
                    A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+1) = (X((9*t)+2) - r_[t](n,2))*dt_;
                    A_x.coeffRef(9*t+6, 3*n_eff_*t+3*n+2) = -(X((9*t)+1) - r_[t](n,1))*dt_;

                    A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+0) = -(X((9*t)+2) - r_[t](n,2))*dt_;
                    A_x.coeffRef(9*t+7, 3*n_eff_*t+3*n+2) = (X((9*t)+0) - r_[t](n,0))*dt_;

                    A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+0) = (X((9*t)+1) - r_[t](n,1))*dt_;
                    A_x.coeffRef(9*t+8, 3*n_eff_*t+3*n+1) = -(X((9*t)+0) - r_[t](n,0))*dt_;
                }
            }
        }
    };

    void CentroidalDynamics::compute_f_mat(Eigen::VectorXd &F){
        for (unsigned t = 0; t < n_col_; ++t){
            A_f.coeffRef(9*t+6, 9*t+1) = -cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_;
            A_f.coeffRef(9*t+6, 9*t+2) = cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_;
            
            A_f.coeffRef(9*t+7, 9*t+0) = cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_;
            A_f.coeffRef(9*t+7, 9*t+2) = -cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_;
            
            A_f.coeffRef(9*t+8, 9*t+0) = -cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_;
            A_f.coeffRef(9*t+8, 9*t+1) = cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_;
            
            b_f[9*t+3] = -cnt_arr_(t,0)*F[3*t*n_eff_+0]*dt_/m_;
            b_f[9*t+4] = -cnt_arr_(t,0)*F[3*t*n_eff_+1]*dt_/m_;
            b_f[9*t+5] = -cnt_arr_(t,0)*F[3*t*n_eff_+2]*dt_/m_ + 9.81*dt_;
            b_f[9*t+6] = (cnt_arr_(t,0)*F[3*t*n_eff_+1]*r_[t](0,2) - cnt_arr_(t,0)*F[3*t*n_eff_+2]*r_[t](0,1))*dt_;
            b_f[9*t+7] = (cnt_arr_(t,0)*F[3*t*n_eff_+2]*r_[t](0,0) - cnt_arr_(t,0)*F[3*t*n_eff_+0]*r_[t](0,2))*dt_;
            b_f[9*t+8] = (cnt_arr_(t,0)*F[3*t*n_eff_+0]*r_[t](0,1) - cnt_arr_(t,0)*F[3*t*n_eff_+1]*r_[t](0,0))*dt_;
            
            for (unsigned n = 1; n < n_eff_; ++n){
                if (cnt_arr_(t,n)) {
                    A_f.coeffRef(9*t+6, 9*t+1) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*dt_;
                    A_f.coeffRef(9*t+6, 9*t+2) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*dt_;
                    
                    A_f.coeffRef(9*t+7, 9*t+0) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*dt_;
                    A_f.coeffRef(9*t+7, 9*t+2) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*dt_;
                    
                    A_f.coeffRef(9*t+8, 9*t+0) += -cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*dt_;
                    A_f.coeffRef(9*t+8, 9*t+1) += cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*dt_;
                    
                    b_f[9*t+3] += -F[3*t*n_eff_+3*n+0]*dt_/m_;
                    b_f[9*t+4] += -F[3*t*n_eff_+3*n+1]*dt_/m_;
                    b_f[9*t+5] += -F[3*t*n_eff_+3*n+2]*dt_/m_;
                    b_f[9*t+6] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+1]*r_[t](n,2) - cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t](n,1))*dt_;
                    b_f[9*t+7] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+2]*r_[t](n,0) - cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t](n,2))*dt_;
                    b_f[9*t+8] += (cnt_arr_(t,n)*F[3*t*n_eff_+3*n+0]*r_[t](n,1) - cnt_arr_(t,0)*F[3*t*n_eff_+3*n+1]*r_[t](n,0))*dt_;
                }
            }
        }
    };

}