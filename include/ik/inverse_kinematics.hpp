//This file contains the implementation of the DDP based IK
// Author : Avadesh Meduri
// Date : 22/04/2021


#ifndef _INVERSE_KINEMATICS_
#define _INVERSE_KINEMATICS_

#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
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

#include "crocoddyl/multibody/residuals/frame-translation.hpp"

#include "ik/action_model.hpp"

// to be removed
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>



namespace ik{

    class InverseKinematics{

        public:
            InverseKinematics(std::string rmodel_path, int n_col);

            void setup_costs(Eigen::VectorXd dt);

            void optimize(const Eigen::VectorXd& x0);

            std::vector<Eigen::VectorXd> get_xs() {return ddp_->get_xs();};
            std::vector<Eigen::VectorXd> get_us() {return ddp_->get_us();};
            Eigen::MatrixXd return_opt_com();
            Eigen::MatrixXd return_opt_mom();


            // cost related functions
            void add_position_tracking_task(pinocchio::FrameIndex fid, int sn, int en, 
                                                Eigen::MatrixXd traj, double wt, std::string cost_name);

            void add_position_tracking_task_single(pinocchio::FrameIndex fid, Eigen::MatrixXd traj,
                                                    double wt, std::string cost_name, int time_step);
            
            void add_terminal_position_tracking_task(pinocchio::FrameIndex fid, Eigen::MatrixXd traj, double wt, 
                                                    std::string cost_name);

            void add_position_tracking_task_single(pinocchio::FrameIndex fid, Eigen::MatrixXd traj,
                                                    Eigen::VectorXd weight, std::string cost_name, int time_step);
            
            void add_terminal_position_tracking_task(pinocchio::FrameIndex fid, Eigen::MatrixXd traj, \
                                                    Eigen::VectorXd weight, std::string cost_name);


            void add_velocity_tracking_task(pinocchio::FrameIndex fid, int sn, int en, 
                                                Eigen::MatrixXd traj, double wt, std::string cost_name);

            void add_com_position_tracking_task(int sn, int en, Eigen::MatrixXd traj, 
                                                Eigen::VectorXd weight, std::string cost_name, bool isTerminal = false);

            void add_centroidal_momentum_tracking_task(int sn, int en, Eigen::MatrixXd traj, 
                                                        Eigen::VectorXd weight, \
                                                        std::string cost_name, bool isTerminal = false);

            void add_state_regularization_cost(int sn, int en, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights, 
                                        Eigen::VectorXd x_reg, bool isTerminal = false);

            void add_state_regularization_cost_single(int time_step, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights, 
                                        Eigen::VectorXd x_reg);

            void add_ctrl_regularization_cost(int sn, int en, double wt, 
                                        std::string cost_name,  Eigen::VectorXd controlWeights, 
                                        Eigen::VectorXd u_reg, bool isTerminal);

            void add_ctrl_regularization_cost_single(int time_step, double wt, 
                                                        std::string cost_name,  Eigen::VectorXd controlWeights, 
                                                        Eigen::VectorXd u_reg);

            void compute_optimal_com_and_mom(Eigen::MatrixXd &pt_com, Eigen::MatrixXd &opt_mom);

        protected:
            //robot mass
            double m_;
            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;
            // number of colocation points
            const int n_col_;
            // crocoddyl state 
            boost::shared_ptr<crocoddyl::StateMultibody> state_;
            // crocoddyl 
            // boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
            boost::shared_ptr<crocoddyl::ActuationModelFull> actuation_;

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

            std::vector<Eigen::VectorXd> xs_;

            // for plotting and kino-dyn
            Eigen::MatrixXd ik_com_opt_;
            Eigen::MatrixXd ik_mom_opt_;
            
    };

    

}


#endif