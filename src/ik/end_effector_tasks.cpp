// This file contains end effector tracking tasks

#include "ik/inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_position_tracking_task(
                    pinocchio::FrameIndex fid, int sn, int en, 
                    Eigen::MatrixXd traj, double wt, std::string cost_name){

        for (unsigned i = sn; i < en; ++i){
            boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
                boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state_, fid, traj));
            rcost_arr_[i].get()->addCost(cost_name + std::to_string(i), goal_tracking_cost, wt);
        }
    };

    void InverseKinematics::add_position_tracking_task_single(pinocchio::FrameIndex fid, Eigen::MatrixXd traj,
        double wt, std::string cost_name, int time_step){

            boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
                    boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                    boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state_, fid, traj));
            rcost_arr_[time_step].get()->addCost(cost_name, goal_tracking_cost, wt);
    };

    void InverseKinematics::add_terminal_position_tracking_task(
                    pinocchio::FrameIndex fid, Eigen::MatrixXd traj, 
                                double wt, std::string cost_name){

        boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
                    boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                    boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state_, fid, traj));
        tcost_model_.get()->addCost(cost_name, goal_tracking_cost, wt);
    };

    void InverseKinematics::add_position_tracking_task_single(pinocchio::FrameIndex fid, Eigen::MatrixXd traj,
        Eigen::VectorXd weight, std::string cost_name, int time_step){
            boost::shared_ptr<crocoddyl::ActivationModelAbstract> goal_activation =
                                        boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(weight);
            boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
                    boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                    goal_activation,
                    boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state_, fid, traj));
            rcost_arr_[time_step].get()->addCost(cost_name, goal_tracking_cost, 1.0);
    };

    void InverseKinematics::add_terminal_position_tracking_task(
                    pinocchio::FrameIndex fid, Eigen::MatrixXd traj, 
                                Eigen::VectorXd weight, std::string cost_name){
                            
        boost::shared_ptr<crocoddyl::ActivationModelAbstract> goal_activation =
                                        boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(weight);
        boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost =
                boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                goal_activation,
                boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state_, fid, traj));
        tcost_model_.get()->addCost(cost_name, goal_tracking_cost, 1.0);
    };

    void InverseKinematics::add_velocity_tracking_task(
                    pinocchio::FrameIndex fid, int st, int et, 
                    Eigen::MatrixXd traj, double wt, std::string cost_name){

        std::cout << "function not implemented" << std::endl;
        // for (unsigned i = sn; i < en; ++i){
        //     crocoddyl::FrameMotion Mref(fid, pinocchio::Motion(traj.row(i - sn)));
        //     boost::shared_ptr<crocoddyl::CostModelAbstract> vel_tracking_cost =
        //         boost::make_shared<crocoddyl::CostModelResidual>(
        //             state_,
        //             boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
        //                 state_, fid, pinocchio::Motion(traj.row(i - sn))));

        //     // boost::shared_ptr<crocoddyl::CostModelAbstract> vel_tracking_cost = 
        //     //     boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, Mref);   

        //     rcost_arr_[i].get()->addCost(cost_name, vel_tracking_cost, wt);
        // };
    };



}