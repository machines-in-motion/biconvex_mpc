// This file contains the biconvex motion planner
// Author : Avadesh Meduri  
// Date : 15/04/2021


#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP

#include "centroidal.hpp"
#include "problem.hpp"


namespace motion_planner
{
class BiConvexMP{

    public:
        
        BiConvexMP(double m, double dt, double T, int n_eff):
        m_(m), centroidal_dynamics(m, dt, T, n_eff){

            // r_.resize(centroidal_dynamics.n_col_)

        };

        // void create_contact_array(cnt_plan);

        void set_contact_plan(std::vector<Eigen::VectorXd> r, Eigen::MatrixXd cnt_plan){
            centroidal_dynamics.r_ = r; centroidal_dynamics.cnt_plan_ = cnt_plan;
        };
    
    private:
        const double m_;
        dynamics::CentroidalDynamics centroidal_dynamics;

    };
}

#endif