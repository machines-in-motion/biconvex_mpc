# include "motion_planner/biconvex.hpp"


namespace motion_planner{

    BiConvexMP::BiConvexMP(double m, int n_col, int n_eff):
        m_(m), n_eff_(n_eff), n_col_(n_col), centroidal_dynamics(m, n_col, n_eff),
        prob_data_x(9, n_col+1), prob_data_f(3*n_eff, n_col),
        fista_x(), fista_f(){
    
            com_opt_.resize(n_col_ + 1, 3); com_opt_.setZero();
            mom_opt_.resize(n_col_ + 1, 6); mom_opt_.setZero();
        
            dyn_violation.resize(9*(n_col_+1));
            dyn_violation.setZero();
            P_k_.resize(9*(n_col_+1));
            P_k_.setZero();

            // setting starting line search params
            fista_x.set_l0(2.25e6);
            fista_f.set_l0(506.25);



            //Use Second Order Cone Projection
            fista_f.set_soc_true();
    };

    void BiConvexMP::create_bound_constraints(Eigen::MatrixXd b, double fx_max, double fy_max, double fz_max){
        
        prob_data_x.lb_ = -1*std::numeric_limits<double>::infinity()*Eigen::VectorXd::Ones(prob_data_x.lb_.size());        
        prob_data_x.ub_ = std::numeric_limits<double>::infinity()*Eigen::VectorXd::Ones(prob_data_x.lb_.size());        
        
        // TODO: Throw errors here
        if (b.cols() != 6){
            std::cout << "bound constraints wrong size. Expected 6 ..." << std::endl;
        }
                
        for (unsigned i = 0; i < centroidal_dynamics.cnt_arr_.rows(); ++i){
            // this can be done once and never done again
            for(unsigned j = 0; j < n_eff_; ++j){
                prob_data_f.lb_[3*n_eff_*i + j*3] = -fx_max;
                prob_data_f.lb_[3*n_eff_*i+ j*3 + 1] = -fy_max;
                prob_data_f.lb_[3*n_eff_*i+ j*3 + 2] = 0;

                prob_data_f.ub_[3*n_eff_*i + j*3] = fx_max;
                prob_data_f.ub_[3*n_eff_*i+ j*3 + 1] = fy_max;
                prob_data_f.ub_[3*n_eff_*i+ j*3 + 2] = fz_max;                    
            }
            if (centroidal_dynamics.cnt_arr_.row(i).sum() > 0){
                prob_data_x.lb_[9*i] = centroidal_dynamics.r_[i].col(0).maxCoeff() + b(i,0);                 
                prob_data_x.lb_[9*i+1] = centroidal_dynamics.r_[i].col(1).maxCoeff() + b(i,1);                 
                prob_data_x.lb_[9*i+2] = centroidal_dynamics.r_[i].col(2).maxCoeff() + b(i,2);

                prob_data_x.ub_[9*i] = centroidal_dynamics.r_[i].col(0).minCoeff() + b(i,3);                 
                prob_data_x.ub_[9*i+1] = centroidal_dynamics.r_[i].col(1).minCoeff() + b(i,4);                 
                prob_data_x.ub_[9*i+2] = centroidal_dynamics.r_[i].col(2).minCoeff() + b(i,5);
            }
        };
    };

    void BiConvexMP::create_cost_X(Eigen::VectorXd W_X, Eigen::VectorXd W_X_ter, Eigen::VectorXd X_ter, Eigen::VectorXd X_nom){

        for (unsigned i = 0; i < prob_data_x.num_vars_ - 9; ++i){
            prob_data_x.Q_.coeffRef(i,i) = W_X[i];
        }
        for (unsigned i = prob_data_x.num_vars_-9; i < prob_data_x.num_vars_ ; ++i){
            prob_data_x.Q_.coeffRef(i,i) = W_X_ter[i - prob_data_x.num_vars_ + 9];
        }
        
        prob_data_x.q_.head(prob_data_x.num_vars_ - 9) = -2*X_nom.cwiseProduct(W_X);
        prob_data_x.q_.tail(9) = -2*X_ter.cwiseProduct(W_X_ter);

    };

    void BiConvexMP::create_cost_F(Eigen::VectorXd W_F){
        for (unsigned i = 0; i < prob_data_f.num_vars_; ++i){
            prob_data_f.Q_.coeffRef(i,i) = W_F[i];
        }
    }

    void BiConvexMP::optimize(Eigen::VectorXd x_init, int num_iters){
        // updating x_init
        centroidal_dynamics.update_x_init(x_init);
        // std::cout << prob_data_f.x_k << std::endl;
        for (unsigned i = 0; i < num_iters; ++i){
            // We need to look into this line...it causes a very high dynamic violation...
            //maxit = init_maxit/(int(i)/10 + 1);

            // std::cout << "optimizing F" << std::endl;
            // optimizing for F
            // std::cout << prob_data_f.x_k.norm() << std::endl;
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);
            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);
            fista_f.optimize(prob_data_f, maxit, tol);

            // std::cout << "optimizing X" << std::endl;
            // optimizing for X
            // std::cout << prob_data_x.x_k.norm() << std::endl;
            centroidal_dynamics.compute_f_mat(prob_data_f.x_k);
            prob_data_x.set_data(centroidal_dynamics.A_f, centroidal_dynamics.b_f, P_k_, rho_);
            fista_x.optimize(prob_data_x, maxit, tol);
            
            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;
            // std::cout << dyn_violation.norm() << std::endl;
            //Keep track of any statistics that may be useful
            if (log_statistics) {
                dyn_violation_hist_.push_back(dyn_violation.norm());
            };

            if(std::isnan(dyn_violation.norm())){
                std::cout << "ERROR: solver diverged, Dyn violation is NaN" << std::endl;
                break;
            };

            if (dyn_violation.norm() < exit_tol){
                // std::cout << "breaking outer loop due to norm ..." << std::endl;
                break;
            };

        }
    
        centroidal_dynamics.r_.clear();
        prob_data_f.x_k *= m_;
        // std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }

    Eigen::MatrixXd BiConvexMP::return_opt_com(){
        for (unsigned i = 0; i < n_col_+1 ; ++i){
            com_opt_(i,0) = prob_data_x.x_k[9*i];
            com_opt_(i,1) = prob_data_x.x_k[9*i+1];
            com_opt_(i,2) = prob_data_x.x_k[9*i+2];
        };
    
        return com_opt_;
    };

    Eigen::MatrixXd BiConvexMP::return_opt_mom(){
        for (unsigned i = 0; i < n_col_ +1; ++i){
            mom_opt_(i,0) =   m_*prob_data_x.x_k[9*i+3];
            mom_opt_(i,1) = m_*prob_data_x.x_k[9*i+4];
            mom_opt_(i,2) = m_*prob_data_x.x_k[9*i+5];
            mom_opt_(i,3) = m_*prob_data_x.x_k[9*i+6]; // de normalizing the angular momentum
            mom_opt_(i,4) = m_*prob_data_x.x_k[9*i+7]; // de normalizing the angular momentum
            mom_opt_(i,5) = m_*prob_data_x.x_k[9*i+8]; // de normalizing the angular momentum
        };
        return mom_opt_;
    };

    void BiConvexMP::update_nomimal_com_mom(Eigen::MatrixXd opt_com, Eigen::MatrixXd opt_mom){

        // Todo : set it up for kino dyn iterations
        // for (unsigned i = 0; i < opt_mom.rows(); ++i){
        //     prob_data_x.Q_(i,i) =         
        // }
    };
};
