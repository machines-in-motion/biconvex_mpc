//This file contains the implementation of the DDP based IK
// Author : Avadesh Meduri
// Date : 22/04/2021


#ifndef _INVERSE_KINEMATICS_
#define _INVERSE_KINEMATICS_

#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include <crocoddyl/core/solver-base.hpp>
#include "crocoddyl/core/optctrl/shooting.hpp"
#include <crocoddyl/core/solvers/ddp.hpp>

#include "crocoddyl/core/activations/weighted-quadratic.hpp"

#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/com-position.hpp"
#include "crocoddyl/multibody/costs/centroidal-momentum.hpp"
#include "crocoddyl/multibody/costs/frame-translation.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/core/costs/control.hpp"

#include "action_model.hpp"



namespace ik{

    class InverseKinematics{

        public:


            InverseKinematics(std::string rmodel_path, double dt, double T);

            void setup_costs();

            void optimize(const Eigen::VectorXd& x0);

            std::vector<Eigen::VectorXd> get_xs() {return ddp_->get_xs();};


            // cost related functions
            void add_position_tracking_task(pinocchio::FrameIndex fid, double st, double et, 
                                                Eigen::MatrixXd traj, double wt, std::string cost_name);
            
            void add_velocity_tracking_task(pinocchio::FrameIndex fid, double st, double et, 
                                                Eigen::MatrixXd traj, double wt, std::string cost_name);

            void add_com_position_tracking_task(double st, double et, Eigen::MatrixXd traj, 
                                                double wt, std::string cost_name, bool isTerminal = false);
            void add_centroidal_momentum_tracking_task(double st, double et, Eigen::MatrixXd traj, 
                                                double wt, std::string cost_name, bool isTerminal = false);

            void add_state_regularization_cost(double st, double et, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights, 
                                        Eigen::VectorXd x_reg);

            void add_ctrl_regularization_cost(double st, double et, double wt, std::string cost_name);


        protected:

            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;
            // discretization
            const double dt_;
            // total horizon length
            const double T_;
            // number of colocation points
            const int n_col_;
            // crocoddyl state 
            boost::shared_ptr<crocoddyl::StateMultibody> state_;
            // crocoddyl 
            boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

            // running cost model array
            std::vector<boost::shared_ptr<crocoddyl::CostModelSum>> rcost_arr_;
            // terminal cost model
            boost::shared_ptr<crocoddyl::CostModelSum> tcost_model_;
            // running integrated action model            
            std::vector< boost::shared_ptr<crocoddyl::ActionModelAbstract>> rint_arr_;
            // terminal intergration action model
            boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> tint_model_;

            // ddp solver
            boost::shared_ptr<crocoddyl::ShootingProblem> problem_;
            boost::shared_ptr<crocoddyl::SolverDDP> ddp_;

            // cost related variables
            int sn;
            int en;
            // boost::shared_ptr<crocoddyl::FrameTranslation> Mref; 



    };
    

}


#endif