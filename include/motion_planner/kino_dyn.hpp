// This file handles the kino dynamic iterations
// Author : Avadesh Meduri
// Date : 21/09/2021

#include "motion_planner/biconvex.hpp"
#include "ik/inverse_kinematics.hpp"

#include "pinocchio/algorithm/centroidal.hpp"

namespace motion_planner{

    class KinoDynMP{

        public:
            KinoDynMP(std::string urdf, int n_eff, int dyn_col, int ik_col);

            BiConvexMP return_dyn(){return dyn;};
            ik::InverseKinematics return_ik(){return ik;};

            void optimize(Eigen::VectorXd q, Eigen::VectorXd v, int dyn_iters, int kino_dyn_iters);

            BiConvexMP dyn;
            ik::InverseKinematics ik;

        private:
            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;

            double m_; // robot mass
            Eigen::VectorXd X_init;
            Eigen::VectorXd X_wm; // warm start X
            Eigen::VectorXd F_wm; // warm start F
            Eigen::VectorXd P_wm; // warm start P
            
            int n = 0; // number of times kino_dyn has been called
    };  
}