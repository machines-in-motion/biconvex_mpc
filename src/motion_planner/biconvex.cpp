# include "motion_planner/biconvex.hpp"


namespace motion_planner{

    BiConvexMP::BiConvexMP(double m, double dt, double T, int n_eff):
        m_(m), n_eff_(n_eff), centroidal_dynamics(m, dt, T, n_eff),
        prob_data_x(9, int(ceil(T/dt))+1), prob_data_f(3*n_eff, int(ceil(T/dt))),
        fista_x(), fista_f(){
            dt_ = dt;
            horizon_ = int(ceil(T/dt_));

            dyn_violation.resize(9*(int (ceil(T/dt))+1));
            dyn_violation.setZero();
            P_k_.resize(9*(int (ceil(T/dt))+1));
            P_k_.setZero();

            // setting starting line search params
            fista_x.set_l0(2.25e6);
            fista_f.set_l0(506.25);

            //Use Second Order Cone Projection
            fista_f.set_soc_true();

            //Set number of variables and constraints for osqp-eigen
            Eigen::SparseMatrix<double> constraintMatF =
                    Eigen::MatrixXd::Identity(3*n_eff_* horizon_, 3*n_eff_* horizon_).sparseView();
            Eigen::SparseMatrix<double> constraintMatX =
                    Eigen::MatrixXd::Identity(9*(horizon_+1), 9*(horizon_+1)).sparseView();

            osqp_x.data()->setNumberOfVariables(9*(horizon_+1));
            osqp_x.data()->setNumberOfConstraints(9*(horizon_+1));

            osqp_x.data()->setLinearConstraintsMatrix(constraintMatX);
            osqp_x.data()->setLowerBound(prob_data_x.lb_);
            osqp_x.data()->setUpperBound(prob_data_x.ub_);
            osqp_x.settings()->setAbsoluteTolerance(1e-5);
            osqp_x.settings()->setRelativeTolerance(1e-5);
            //osqp_x.settings()->setMaxIteration(25);
            osqp_x.settings()->setScaling(0);
            osqp_x.settings()->setWarmStart(true);
            osqp_x.settings()->setVerbosity(false);

            osqp_f.data()->setNumberOfVariables(3*n_eff_* horizon_);
            osqp_f.data()->setNumberOfConstraints(3*n_eff_*horizon_);
            osqp_f.data()->setLinearConstraintsMatrix(constraintMatF);
            osqp_f.data()->setLowerBound(prob_data_f.lb_);
            osqp_f.data()->setUpperBound(prob_data_f.ub_);
            osqp_f.settings()->setAbsoluteTolerance(1e-5);
            osqp_f.settings()->setRelativeTolerance(1e-5);
            //osqp_f.settings()->setMaxIteration(25);
            osqp_f.settings()->setScaling(0);
            osqp_f.settings()->setWarmStart(true);
            osqp_f.settings()->setVerbosity(false);
    };

    void BiConvexMP::optimize(Eigen::VectorXd x_init, int num_iters){
        // updating x_init
        centroidal_dynamics.update_x_init(x_init);

        for (unsigned i = 0; i < num_iters; ++i){
            // We need to look into this line...it causes a very high dynamic violation...
            //maxit = init_maxit/(int(i)/10 + 1);

            // optimizing for F
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);
            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);
            fista_f.optimize(prob_data_f, maxit, tol);

            // optimizing for X
            centroidal_dynamics.compute_f_mat(prob_data_f.x_k);
            prob_data_x.set_data(centroidal_dynamics.A_f, centroidal_dynamics.b_f, P_k_, rho_);
            fista_x.optimize(prob_data_x, maxit, tol);

            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;
            //Keep track of any statistics that may be useful
            if (log_statistics) {
                dyn_violation_hist_.push_back(dyn_violation.norm());
                std::cout << dyn_violation.norm() << std::endl;
            }

            if (dyn_violation.norm() < exit_tol){
                // std::cout << "breaking outer loop due to norm ..." << std::endl;
                break;
            };
        }
        // std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }

    void BiConvexMP::optimize_osqp(Eigen::VectorXd x_init, int num_iters){
        // updating x_init
        centroidal_dynamics.update_x_init(x_init);

        for (unsigned i = 0; i < num_iters; ++i){
            // optimizing for F
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);
            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);

            if (i == 0) {
                osqp_f.data()->setHessianMatrix(prob_data_f.ATA_);
                osqp_f.data()->setGradient(prob_data_f.ATbPk_);
                osqp_f.initSolver();
            }
            else {
                osqp_f.updateHessianMatrix(prob_data_f.ATA_);
                osqp_f.updateGradient(prob_data_f.ATbPk_);
            }
            osqp_f.solve();
            prob_data_f.x_k = osqp_f.getSolution();

            // optimizing for X
            centroidal_dynamics.compute_f_mat(prob_data_f.x_k);
            prob_data_x.set_data(centroidal_dynamics.A_f, centroidal_dynamics.b_f, P_k_, rho_);

            if (i == 0) {
                osqp_x.data()->setHessianMatrix(prob_data_x.ATA_);
                osqp_x.data()->setGradient(prob_data_x.ATbPk_);
                osqp_x.initSolver();
            }
            else {
                osqp_x.updateHessianMatrix(prob_data_x.ATA_);
                osqp_x.updateGradient(prob_data_x.ATbPk_);
            }
            osqp_x.solve();
            prob_data_x.x_k = osqp_x.getSolution();

            //Calculate dynamic violation
            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;

            //Keep track of any statistics that may be useful
            if (log_statistics) {
                dyn_violation_hist_.push_back(dyn_violation.norm());
                std::cout << dyn_violation.norm() << std::endl;
            }

            if (dyn_violation.norm() < exit_tol){
                // std::cout << "breaking outerloop due to norm ..." << std::endl;
                //std::cout << "Optimizer finished after " << i << " iterations" << std::endl;
                break;
            };
        }
        //std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }

    void BiConvexMP::update_cost_x(Eigen::VectorXd X_ter, Eigen::VectorXd X_ter_nrml) {
        //Change for loop to prob_data_x.num_vars_ - (2*prob_data.x.state)
        //TODO: See if you can user the Eigen Sparse innerloop functionality to update faster...

        //First loop: Loop through all of horizon except the last two knot points
        for (unsigned int i = 0; i < (prob_data_x.num_vars_-(2*9))/9; ++i) {
            prob_data_x.Q_.coeffRef(i,i) = prob_data_x.Q_.coeffRef(i+9, i+9);
            prob_data_x.q_[i] = prob_data_x.q_[i+9];
        } 

        //Second loop: Loop through second to last knot point: 
        //TODO: THis could also change to dividing this cost by some weighting factor (i.e. if the final cost is
        // just multifplied by a weighting factor, we can divide by this weighting factor)
        for (unsigned int i = 0; i < 9; ++i) {
            prob_data_x.Q_.coeffRef(prob_data_x.num_vars_-(2*9)+i,prob_data_x.num_vars_-(2*9)+i) = X_ter_nrml[i];
            prob_data_x.q_[prob_data_x.num_vars_-(2*9)+i] = X_ter_nrml[i];
        }

        //Set Terminal Constraint (last knot point): 
        for (unsigned int i = 0; i < 9; ++i) {
            prob_data_x.Q_.coeffRef(prob_data_x.num_vars_-(9)+i, prob_data_x.num_vars_-(9)+i) = X_ter[i];
            prob_data_x.q_[prob_data_x.num_vars_-(9)+i] = X_ter[i];
        }
    }

    void BiConvexMP::update_bounds_x(Eigen::VectorXd lb_fin, Eigen::VectorXd ub_fin) {
        //Shift bounds
        //prob_data_x.lb_.segment(1, prob_data_x.num_vars_-9) = prob_data_x.lb_.segment(9, prob_data_x.num_vars_)
        //prob_data_x.ub_.segment(1, prob_data_x.num_vars_-9) = prob_data_x.ub_.segment(9, prob_data_x.num_vars_)

        prob_data_x.lb_.head(prob_data_x.num_vars_-9) = prob_data_x.lb_.tail(prob_data_x.num_vars_-9);
        prob_data_x.ub_.head(prob_data_x.num_vars_-9) = prob_data_x.ub_.tail(prob_data_x.num_vars_-9);

        //Update new bounds (end of bound vectors)
        prob_data_x.lb_.tail(9) = lb_fin;
        prob_data_x.ub_.tail(9) = ub_fin;
    }

    // void BiConvexMP::update_constraints_x() {
    //     for (unsigned int t = 0; t < centroidal_dynamics.n_col_-1; ++t) {
    //         centroidal_dynamics.A_f.coeffRef(9*t+6, 9*t+1) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+6, 9*(t+1)+1);
    //         centroidal_dynamics.A_f.coeffRef(9*t+6, 9*t+2) = centroidal_dynamics.A_f.coeffRef(9*(t+2)+6, 9*(t+2)+9);
            
    //         centroidal_dynamics.A_f.coeffRef(9*t+7, 9*t+0) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+7, 9*(t+1)+0);
    //         centroidal_dynamics.A_f.coeffRef(9*t+7, 9*t+2) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+7, 9*(t+1)+2);
            
    //         centroidal_dynamics.A_f.coeffRef(9*t+8, 9*t+0) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+8, 9*(t+1)+0);
    //         centroidal_dynamics.A_f.coeffRef(9*t+8, 9*t+1) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+8, 9*(t+1)+1);

    //         //TODO: Use eigen::segment functionality
    //         centroidal_dynamics.b_f[9*t+3] = centroidal_dynamics.b_f[9*(t+1)+3];
    //         centroidal_dynamics.b_f[9*t+4] = centroidal_dynamics.b_f[9*(t+1)+4];
    //         centroidal_dynamics.b_f[9*t+5] = centroidal_dynamics.b_f[9*(t+1)+5];
    //         centroidal_dynamics.b_f[9*t+6] = centroidal_dynamics.b_f[9*(t+1)+6];
    //         centroidal_dynamics.b_f[9*t+7] = centroidal_dynamics.b_f[9*(t+1)+7];
    //         centroidal_dynamics.b_f[9*t+8] = centroidal_dynamics.b_f[9*(t+1)+8];
    //     }

    //     //Update final constraint
    //     auto last_state = centroidal_dynamics.n_col_ - 1;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 1) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 2) = 0.0;
        
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 0) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 2) = 0.0;
        
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 0) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 1) = 0.0;

    //     centroidal_dynamics.b_f.tail(9) = Eigen::VectorXd::Zero(9);

    //     //If any of the incoming contacts are in contact
    //     if (centroidal_dynamics.cnt_arr_.row(centroidal_dynamics.n_col_).sum() > 0) {
    //         for (unsigned int i = 0; i < centroidal_dynamics.n_eff_; ++i) {
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 1) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 2) = 0.0;
                
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 0) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 2) = 0.0;
                
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 0) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 1) = 0.0;

    //             centroidal_dynamics.b_f.tail(9) = Eigen::VectorXd::Zero(9);
    //         }
    //     }
    // }

    void BiConvexMP::shift_horizon() {
        //Update new contacts

        //update X constraints

        //update bounds for state

        //update X cost function

        //Update previous solutions
        if (use_prev_soln) {
            prob_data_x.x_k.head(prob_data_x.num_vars_ - prob_data_x.state_) = \
                prob_data_x.x_k.tail(prob_data_x.num_vars_ - prob_data_x.state_);
            prob_data_x.x_k.tail(prob_data_x.state_) = Eigen::VectorXd::Zero(prob_data_x.state_);

            prob_data_f.x_k.head(prob_data_f.num_vars_ - prob_data_f.state_) = \
                prob_data_f.x_k.tail(prob_data_f.num_vars_ - prob_data_f.state_);
            prob_data_f.x_k.tail(prob_data_f.state_) = Eigen::VectorXd::Zero(prob_data_f.state_);

            P_k_.head(9*(int(horizon_/dt_))) = P_k_.tail(9*(int(horizon_/dt_)));
            P_k_.tail(9) = Eigen::VectorXd::Zero(9);
        }
        else {
            prob_data_x.x_k.setZero();
            prob_data_f.x_k.setZero();
            P_k_.setZero();
        }
    }

};
