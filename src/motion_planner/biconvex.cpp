# include "motion_planner/biconvex.hpp"
# include <chrono>

namespace motion_planner{

    BiConvexMP::BiConvexMP(double m, int n_col, int n_eff):
        m_(m), n_eff_(n_eff), n_col_(n_col), centroidal_dynamics(m, n_col, n_eff),
        prob_data_x(9, n_col+1), prob_data_f(3*n_eff, n_col),
        fista_x(), fista_f() {
            com_opt_.resize(n_col_ + 1, 3);
            com_opt_.setZero();
            mom_opt_.resize(n_col_ + 1, 6);
            mom_opt_.setZero();

            //Number of State Constraints
            num_com_states_ = 9*(n_col_+1);
            std::cout << "Number of State Constraints: " << num_com_states_ << std::endl;

            //Number of Friction Constraints
            num_friction_constraints_ = 5*n_eff_*n_col_;

            if (variable_footsteps_) {
                prob_data_x.resize((9+(3*n_eff_))*(n_col_+1));
                centroidal_dynamics.resize_f_mat(9+(3*n_eff_));
                centroidal_dynamics.set_variable_footsteps(true);
                std::cout << "Prob Data Resized for variable footsteps" << std::endl;
            }

            dyn_violation.resize(prob_data_x.num_vars_);
            dyn_violation.setZero();
            P_k_.resize(prob_data_x.num_vars_);
            P_k_.setZero();

            // FISTA specific parameters/setup TODO: Move this into another class?
            // setting starting line search params for FISTA
            // TODO: Move this into FISTA
            fista_x.set_l0(2.25e6);
            fista_f.set_l0(506.25);

            //Use Second Order Cone Projection for FISTA
            fista_f.set_soc_true();

            // OSQP Specific Setup
            #ifdef USE_OSQP
                std::cout << "Using Osqp" << std::endl;
                // Set number of variables and constraints for osqp
                Eigen::SparseMatrix<double> constraintMatX =
                        Eigen::MatrixXd::Identity(prob_data_x.num_vars_, prob_data_x.num_vars_).sparseView();

                osqp_x.data()->setNumberOfVariables(prob_data_x.num_vars_);
                osqp_x.data()->setNumberOfConstraints(prob_data_x.num_vars_);

                osqp_x.data()->setLinearConstraintsMatrix(constraintMatX);
                osqp_x.data()->setLowerBound(prob_data_x.lb_);
                osqp_x.data()->setUpperBound(prob_data_x.ub_);
                osqp_x.settings()->setAbsoluteTolerance(1e-5);
                osqp_x.settings()->setRelativeTolerance(1e-5);
                //osqp_x.settings()->setMaxIteration(10);
                osqp_x.settings()->setScaling(1);
                osqp_x.settings()->setWarmStart(true);
                osqp_x.settings()->setVerbosity(false);

                osqp_f.data()->setNumberOfVariables(3*n_eff_* n_col_);

                if (use_proper_constraints_) {
                    Eigen::SparseMatrix<double> constraintMatFTrue =
                        Eigen::MatrixXd::Zero(num_friction_constraints_, prob_data_f.num_vars_).sparseView();

                    //Set up friction cone constraints properly
                    for (unsigned int i = 0; i < n_col_; ++i) {
                        for (unsigned int j = 0; j < n_eff_; ++j) {
                            //Friction Cone Constraints on Fx (i.e. mu*Fz <= Fx <= mu*Fz)
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 0, i*(n_eff_*3) + 3*j + 0) = 1.0;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 0, i*(n_eff_*3) + 3*j + 2) = mu_;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 1, i*(n_eff_*3) + 3*j + 0) = 1.0;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 1, i*(n_eff_*3) + 3*j + 2) = -mu_;

                            //Friction Cone Constraints on Fy (i.e. mu*Fz <= Fy <= mu*Fz)
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 2, i*(n_eff_*3) + 3*j + 1) = 1.0;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 2, i*(n_eff_*3) + 3*j + 2) = mu_;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 3, i*(n_eff_*3) + 3*j + 1) = 1.0;
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 3, i*(n_eff_*3) + 3*j + 2) = -mu_;

                            //Friction Cone Constraints on Fz (i.e. 0 <= Fz <= Fz_max)
                            constraintMatFTrue.coeffRef(i*(n_eff_*5) + 5*j + 4, i*(n_eff_*3) + 3*j + 2) = 1.0;
                        }
                    }

                    osqp_f.data()->setNumberOfConstraints(5*n_eff_* n_col_);
                    osqp_f.data()->setLinearConstraintsMatrix(constraintMatFTrue);
                }
                else {
                    Eigen::SparseMatrix<double> constraintMatF =
                        Eigen::MatrixXd::Identity(prob_data_f.num_vars_, prob_data_f.num_vars_).sparseView();

                    osqp_f.data()->setNumberOfConstraints(3*n_eff_* n_col_);
                    osqp_f.data()->setLinearConstraintsMatrix(constraintMatF);
                }

                osqp_f.data()->setLowerBound(prob_data_f.lb_);
                osqp_f.data()->setUpperBound(prob_data_f.ub_);
                osqp_f.settings()->setAbsoluteTolerance(1e-5);
                osqp_f.settings()->setRelativeTolerance(1e-5);
                //osqp_f.settings()->setMaxIteration(25);
                osqp_f.settings()->setScaling(1);
                osqp_f.settings()->setWarmStart(true);
                osqp_f.settings()->setVerbosity(false);
            #endif
    };

    void BiConvexMP::create_bound_constraints(Eigen::MatrixXd b, double fx_max, double fy_max, double fz_max){
        // Add checks to use regular infinity and not osqp infinity when using FISTA
//        prob_data_x.lb_ = -1*OsqpEigen::INFTY*Eigen::VectorXd::Ones(prob_data_x.lb_.size());
//        prob_data_x.ub_ = OsqpEigen::INFTY*Eigen::VectorXd::Ones(prob_data_x.lb_.size());

        prob_data_x.lb_ = -1*100000000.0*Eigen::VectorXd::Ones(prob_data_x.lb_.size());
        prob_data_x.ub_ = 100000000.0*Eigen::VectorXd::Ones(prob_data_x.lb_.size());

        if (variable_footsteps_) {
            prob_data_x.lb_.tail(3 * n_eff_ * (prob_data_x.horizon_)) = Eigen::VectorXd::Zero(
                    3 * n_eff_ * (prob_data_x.horizon_));
            prob_data_x.ub_.tail(3 * n_eff_ * (prob_data_x.horizon_)) = Eigen::VectorXd::Zero(
                    3 * n_eff_ * (prob_data_x.horizon_));
        }
        
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
                prob_data_x.lb_[9*i] =   centroidal_dynamics.r_[i].col(0).maxCoeff() + b(i,0);
                prob_data_x.lb_[9*i+1] = centroidal_dynamics.r_[i].col(1).maxCoeff() + b(i,1);
                prob_data_x.lb_[9*i+2] = centroidal_dynamics.r_[i].col(2).maxCoeff() + b(i,2);

                prob_data_x.ub_[9*i] =   centroidal_dynamics.r_[i].col(0).minCoeff() + b(i,3);
                prob_data_x.ub_[9*i+1] = centroidal_dynamics.r_[i].col(1).minCoeff() + b(i,4);
                prob_data_x.ub_[9*i+2] = centroidal_dynamics.r_[i].col(2).minCoeff() + b(i,5);
            }
        }
    };

    void BiConvexMP::create_cost_X(Eigen::VectorXd W_X, Eigen::VectorXd W_X_ter, Eigen::VectorXd X_ter, Eigen::VectorXd X_nom){
        //Set Hessians

        for (unsigned int i = 0; i < num_com_states_ - 9; ++i){
            prob_data_x.Q_.coeffRef(i,i) = W_X[i];
        }

        //Fix this...it's ugly
        int j = 0;
        std::cout << "initial i: " <<  num_com_states_ - 9 << std::endl;
        for (unsigned int i = num_com_states_ - 9; i < num_com_states_; ++i){
            prob_data_x.Q_.coeffRef(i,i) = W_X_ter[j];
            ++j;
        }

        if (variable_footsteps_) {
            for (unsigned int i = num_com_states_; i < prob_data_x.num_vars_; ++i) {
                prob_data_x.Q_.coeffRef(i, i) = 1e-8;
            }
        }

        //Set Gradients

        prob_data_x.q_.head(9*(n_col_+1)) = -2*X_nom.cwiseProduct(W_X);
        //prob_data_x.q_.tail(9) = -2*X_ter.cwiseProduct(W_X_ter);
        prob_data_x.q_.segment(9*(n_col_), 9) = -2*X_ter.cwiseProduct(W_X_ter);
    };

    void BiConvexMP::create_cost_F(Eigen::VectorXd W_F) {
        for (unsigned int i = 0; i < prob_data_f.num_vars_; ++i){
            prob_data_f.Q_.coeffRef(i,i) = W_F[i];
        }
    }

    void BiConvexMP::update_initial_states(Eigen::VectorXd init_constraints) {
        //Add assert to make sure the Vector provided has the same length as the initial state constraints
        for (int i = 0; i < 9; ++i) {
            prob_data_x.lb_[i] = init_constraints[i];
            prob_data_x.ub_[i] = init_constraints[i];
        }
    }

    void BiConvexMP::optimize(Eigen::VectorXd x_init, int num_iters) {
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

            //Dynamic Violation Calculation
            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;
            // std::cout << dyn_violation.norm() << std::endl;

            //Keep track of any statistics that may be useful
            if (log_statistics) {
                dyn_violation_hist_.push_back(dyn_violation.norm());
                std::cout << dyn_violation.norm() << std::endl;
            };

            if(std::isnan(dyn_violation.norm())){
                std::cout << "ERROR: solver diverged, Dyn violation is NaN" << std::endl;
                break;
            };

            if (dyn_violation.norm() < exit_tol){
                std::cout << "breaking outer loop due to norm ..." << std::endl;
                break;
            };
        }
    
        centroidal_dynamics.r_.clear();
        std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }

    #ifdef USE_OSQP
    void BiConvexMP::optimize_osqp(Eigen::VectorXd x_init, int num_iters){

        // updating x_init
        centroidal_dynamics.update_x_init(x_init);

        for (unsigned i = 0; i < num_iters; ++i){
            // optimizing for F
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);

            std::cout << "Setting Data" << std::endl;

            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);

            std::cout << "Finished Setting Data" << std::endl;

            if (i == 0) {
                std::cout << "setting hessian" << std::endl;

                osqp_f.data()->setHessianMatrix(prob_data_f.ATA_);

                std::cout << "setting gradient" << std::endl;

                osqp_f.data()->setGradient(prob_data_f.ATbPk_);

                std::cout << "initializing solver" << std::endl;
                osqp_f.initSolver();
            }
            else {
                osqp_f.updateHessianMatrix(prob_data_f.ATA_);
                osqp_f.updateGradient(prob_data_f.ATbPk_);
            }

            std::cout << "Solving Problem" << std::endl;

            osqp_f.solveProblem();

            std::cout << "Solved Problem" << std::endl;

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

            osqp_x.solveProblem();
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
                std::cout << "breaking outerloop due to norm ..." << std::endl;

                //Clear Contact Array (Check why this is necessary)
                centroidal_dynamics.r_.clear();

                //Clear solver (can I get rid of the clearSolver?)
                osqp_f.clearSolver();
                osqp_f.data()->clearHessianMatrix();
                osqp_x.clearSolver();
                osqp_x.data()->clearHessianMatrix();
                std::cout << "Optimizer finished after " << i << " iterations" << std::endl;
                break;
            };
        }

        //Clear Contact Array (Check why this is necessary)
        centroidal_dynamics.r_.clear();

        //Check if we can do things without clearing the Solvers and Hessians
        osqp_f.clearSolver();
        osqp_f.data()->clearHessianMatrix();
        osqp_x.clearSolver();
        osqp_x.data()->clearHessianMatrix();
        //std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }
    #endif

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
            mom_opt_(i,3) = prob_data_x.x_k[9*i+6];
            mom_opt_(i,4) = prob_data_x.x_k[9*i+7];
            mom_opt_(i,5) = prob_data_x.x_k[9*i+8];
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
