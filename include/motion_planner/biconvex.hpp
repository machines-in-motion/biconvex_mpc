// This file contains the biconvex motion planner
// Author : Avadesh Meduri
// Date : 15/04/2021

#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP

#ifdef USE_OSQP
#include "OsqpEigen/OsqpEigen.h"
#endif

#include <stdio.h> 
#include <iostream>

#include "dynamics/centroidal.hpp"
#include "solvers/problem.hpp"
#include "solvers/fista.hpp"

namespace motion_planner
{
class BiConvexMP{
    public:
        BiConvexMP(double m, int n_col, int n_eff);

        void set_contact_plan(Eigen::MatrixXd cnt_plan, double dt){
            centroidal_dynamics.set_contact_arrays(cnt_plan, dt);
        };


        Eigen::MatrixXd return_A_x(Eigen::VectorXd X){
            centroidal_dynamics.compute_x_mat(X);  
            return centroidal_dynamics.A_x;
        }
    
        Eigen::MatrixXd return_b_x(Eigen::VectorXd X){
            centroidal_dynamics.compute_x_mat(X);  
            return centroidal_dynamics.b_x;
        }
    

        Eigen::MatrixXd return_A_f(Eigen::VectorXd F, Eigen::VectorXd x_init){
            centroidal_dynamics.compute_f_mat(F);  
            centroidal_dynamics.update_x_init(x_init);
            return centroidal_dynamics.A_f;
        }

        Eigen::MatrixXd return_b_f(Eigen::VectorXd F, Eigen::VectorXd x_init){
            centroidal_dynamics.compute_f_mat(F);  
            centroidal_dynamics.update_x_init(x_init);
            return centroidal_dynamics.b_f;
        }
    
        // function to set cost function
        void set_cost_x(Eigen::SparseMatrix<double> Q_x, Eigen::VectorXd q_x){
            prob_data_x.Q_ = Q_x; prob_data_x.q_ = q_x;
        }

        void set_cost_f(Eigen::SparseMatrix<double> Q_f, Eigen::VectorXd q_f){
            prob_data_f.Q_ = Q_f; prob_data_f.q_ = q_f;
        }
        
        void set_rho(double rho){
            rho_ = rho;
        }
        
        void set_warm_start_vars(Eigen::VectorXd x_wm, Eigen::VectorXd f_wm, Eigen::VectorXd P_wm){
            prob_data_x.set_warm_x(x_wm);
            prob_data_f.set_warm_x(f_wm);
            P_k_ = P_wm;
        }

        void set_bounds_x(Eigen::VectorXd lb, Eigen::VectorXd ub) 
                {prob_data_x.lb_ = lb; prob_data_x.ub_ = ub;}

        void set_bounds_f(Eigen::VectorXd lb, Eigen::VectorXd ub) 
                {prob_data_f.lb_ = lb; prob_data_f.ub_ = ub;}

        // box constraints created on parameters and contact plan
        void create_bound_constraints(Eigen::MatrixXd b, double fx_max, double fy_max, double fz_max);
        // creates basic quadratic costs for optimizing X
        void create_cost_X(Eigen::VectorXd W_X, Eigen::VectorXd W_X_ter, Eigen::VectorXd X_ter, Eigen::VectorXd X_nom);
        void create_cost_F(Eigen::VectorXd W_F);
        void update_nomimal_com_mom(Eigen::MatrixXd opt_com, Eigen::MatrixXd opt_mom);
 
        void set_rotation_matrix_f(Eigen::MatrixXd rot_matrix)
        {
            prob_data_f.rotation_matrices.push_back(rot_matrix);
            prob_data_f.rotation_matrices_trans.push_back(rot_matrix.transpose());
        }


        void optimize(Eigen::VectorXd x_init, int no_iters);

        void optimize_osqp(Eigen::VectorXd x_init, int no_iters);

        //Shifting cost function for MPC by one knot point
        //TODO: Rename to shift_cost() ?
        void update_cost_x(Eigen::VectorXd X_ter, Eigen::VectorXd X_ter_nrml);

        //Update bounds on X (states) for MPC
        //Inputs: lb_fin = new final lower bound constraints, not ALL bounds (i.e. should be length = 9)
        //      : ub_fin = new final upper bound constraints, not ALL bounds (i.e. should be length 
        void update_bounds_x(Eigen::VectorXd lb_fin, Eigen::VectorXd ub_fin);

        //Update constraint Matrix A_x, and b_x
        void update_constraints_x();

        //Shift cost functions, constraints, and solutions
        void shift_horizon();
        

        Eigen::VectorXd return_opt_x(){
            return prob_data_x.x_k;
        }

        Eigen::VectorXd return_opt_f(){
            return prob_data_f.x_k;
        }

        Eigen::VectorXd return_opt_p(){
            return P_k_;
        }

        Eigen::MatrixXd return_opt_com();
        Eigen::MatrixXd return_opt_mom();

        std::vector<double> return_dyn_viol_hist(){
            return dyn_violation_hist_;
        }

        void set_friction_coefficient(double mu) {
            fista_f.set_friction_coefficient(mu);
        }

        void set_robot_mass(double m) {
            m_ = m;
        };
        
        void collect_statistics(){log_statistics = 1;};

        // mass of the robot 
        double m_;

    private:
        // centroidal dynamics class
        dynamics::CentroidalDynamics centroidal_dynamics;
        // penalty term on dynamic violation
        double rho_ = 1e+5;
        // initial step length
        double L0_ = 1e2;
        // line search parameter
        double beta_ = 1.5;
        // max iters in Fista
        int init_maxit = 150;
        // max iters in Fista reduced based on outer loops
        int maxit = 150;
        // tolerance for exit criteria of Fista
        double tol = 1e-5;
        // tolerance for exiting biconvex
        double exit_tol = 1e-3;

        int n_col_ = 0;
        int n_eff_ = 0;
        double T_;
        
        // problem data for x optimization (Used in optimization for Forces)
        function::ProblemData prob_data_x;
        // solver for x optimization
        solvers::FISTA fista_x;

        // problem data for f optimization (Used in optimization for CoM, Vel, Amom)
        function::ProblemData prob_data_f;
        // solver for f optimization
        solvers::FISTA fista_f;

        // optimal CoM and Momentum trajectory (required for IK)
        Eigen::MatrixXd com_opt_;
        Eigen::MatrixXd mom_opt_;
        
        #ifdef USE_OSQP
            OsqpEigen::Solver osqp_x;
            OsqpEigen::Solver osqp_f;
        #endif

        Eigen::VectorXd dyn_violation;
        Eigen::VectorXd P_k_;

        bool use_prev_soln = false;

        bool log_statistics = false;
        std::vector<double> dyn_violation_hist_;

    };
}

#endif