// This file contains the biconvex motion planner
// Author : Avadesh Meduri  
// Date : 15/04/2021


#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP
#include <iostream>
#include "centroidal.hpp"
#include "problem.hpp"


namespace motion_planner
{
class BiConvexMP{

    public:
        
        BiConvexMP(double m, double dt, double T, int n_eff):
        m_(m), centroidal_dynamics(m, dt, T, n_eff){

        };

        void set_contact_plan(Eigen::MatrixXd cnt_plan){
            centroidal_dynamics.cnt_plan_.push_back(cnt_plan);
        };

        void return_cnt_plan(){
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
    

    private:
        // mass of the robot 
        const double m_;
        // centroidal dynamics class
        dynamics::CentroidalDynamics centroidal_dynamics;
    

    };
}

#endif