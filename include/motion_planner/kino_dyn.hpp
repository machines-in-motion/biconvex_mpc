// This file handles the kino dynamic iterations
// Author : Avadesh Meduri
// Date : 21/09/2021
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "motion_planner/biconvex.hpp"
#include "ik/inverse_kinematics.hpp"

#include "pinocchio/algorithm/centroidal.hpp"
#include <stdio.h>

namespace motion_planner{

    class KinoDynMP{

        public:
            KinoDynMP(std::string urdf, double m, int n_eff, int dyn_col, int ik_col);

            BiConvexMP* return_dyn(){return &dyn;};
            ik::InverseKinematics* return_ik(){return &ik;};

            void set_warm_starts();
            void optimize(Eigen::VectorXd q, Eigen::VectorXd v, int dyn_iters, int kino_dyn_iters);

            void set_com_tracking_weight(Eigen::VectorXd wt_com){wt_com_ = wt_com;};
            void set_mom_tracking_weight(Eigen::VectorXd wt_mom){wt_mom_ = wt_mom;};
            void compute_solve_times(){profile_code = 1;};
            Eigen::VectorXd return_solve_times(){return solve_times;};

            BiConvexMP dyn;
            ik::InverseKinematics ik;


        private:
            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;

            double m_; // robot mass
            Eigen::VectorXd X_init;
            Eigen::VectorXd x0;
            Eigen::VectorXd X_wm; // warm start X
            Eigen::VectorXd F_wm; // warm start F
            Eigen::VectorXd P_wm; // warm start P
            
            int n = 0; // number of times kino_dyn has been called
            int dyn_col_;
            int ik_col_;

            // optimal com and mom trajectories
            Eigen::MatrixXd dyn_com_opt;
            Eigen::MatrixXd dyn_mom_opt;

            Eigen::MatrixXd ik_com_opt;
            Eigen::MatrixXd ik_mom_opt;

            Eigen::VectorXd wt_com_; // com tracking weight
            Eigen::VectorXd wt_mom_; // momentum tracking weight

            // profiling the code
            bool profile_code = 0;
            Eigen::VectorXd solve_times;
    };  
}