// This file contains the biconvex motion planner
// Author : Avadesh Meduri
// Date : 15/04/2021

#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP

#ifdef USE_OSQP
#include "OsqpEigen/OsqpEigen.h"
#endif

#include <iostream>

#include "dynamics/centroidal.hpp"
#include "solvers/problem.hpp"
#include "solvers/fista.hpp"

namespace motion_planner
{
class BiConvexMP{
    public:
        BiConvexMP(double m, double dt, double T, int n_eff);

        void set_contact_plan(Eigen::MatrixXd cnt_plan){
            centroidal_dynamics.cnt_plan_.push_back(cnt_plan);
        };

        void create_cnt_array(){
            centroidal_dynamics.create_contact_array();  
        }

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
        
        void set_warm_start_vars(Eigen::VectorXd x_wm, Eigen::VectorXd f_wm, Eigen::VectorXd P_wm){
            prob_data_x.set_warm_x(x_wm);
            prob_data_f.set_warm_x(f_wm);
            P_k_ = P_wm;
        }

        void set_bounds_x(Eigen::VectorXd lb, Eigen::VectorXd ub) 
                {prob_data_x.lb_ = lb; prob_data_x.ub_ = ub;}

        void set_bounds_f(Eigen::VectorXd lb, Eigen::VectorXd ub) 
                {prob_data_f.lb_ = lb; prob_data_f.ub_ = ub;}

        void optimize(Eigen::VectorXd x_init, int no_iters);

        //Shifting cost function for MPC
        //TODO: Rename to shift_cost() ?
        void update_cost_x(Eigen::VectorXd X_ter, Eigen::VectorXd X_ter_nrml);

        //Update bounds on X (states) for MPC
        //Inputs: lb_fin = new final lower bound constraints, not ALL bounds (i.e. should be length = 9)
        //      : ub_fin = new final upper bound constraints, not ALL bounds (i.e. should be length 
        void update_bounds_x(Eigen::VectorXd lb_fin, Eigen::VectorXd ub_fin);

        //Update constraint Matrix A_x, and b_x
        void update_constraints_x();
        

        Eigen::VectorXd return_opt_x(){
            return prob_data_x.x_k;
        }

        Eigen::VectorXd return_opt_f(){
            return prob_data_f.x_k;
        }

        Eigen::VectorXd return_opt_p(){
            return P_k_;
        }

    private:
        // mass of the robot 
        const double m_;
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
        // problem data for x optimization
        function::ProblemData prob_data_x;
        // solver for x optimization
        solvers::FISTA fista_x;
        // problem data for f optimization
        function::ProblemData prob_data_f;
        // solver for f optimization
        solvers::FISTA fista_f;

        #ifdef USE_OSQP
            OsqpEigen::Solver osqp_x;
            OsqpEigen::Solver osqp_f;
        #endif

        Eigen::VectorXd dyn_violation;

        Eigen::VectorXd P_k_;

    };
}

#endif