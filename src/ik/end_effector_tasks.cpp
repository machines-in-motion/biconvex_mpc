// This file contains end effector tracking tasks

#include "ik/inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_position_tracking_task(
                    pinocchio::FrameIndex fid, double st, double et, 
                    Eigen::MatrixXd traj, double wt, std::string cost_name){

        sn = st/dt_; 
        en = et/dt_;

        if (et - st == 0){
            crocoddyl::FrameTranslation Mref(fid, traj);
            boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost = 
                boost::make_shared<crocoddyl::CostModelFrameTranslation>(state_, Mref);   

            rcost_arr_[sn].get()->addCost(cost_name, goal_tracking_cost, wt);
        }
        else{
            for (unsigned i = sn; i < en; ++i){
                crocoddyl::FrameTranslation Mref(fid, traj.row(i));
                boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost = 
                    boost::make_shared<crocoddyl::CostModelFrameTranslation>(state_, Mref);   

                rcost_arr_[i].get()->addCost(cost_name, goal_tracking_cost, wt);
            }
        };
    };

    void InverseKinematics::add_velocity_tracking_task(
                    pinocchio::FrameIndex fid, double st, double et, 
                    Eigen::MatrixXd traj, double wt, std::string cost_name){

        // This function does not work properly yet

        sn = st/dt_; 
        en = et/dt_;

        // if (et - st == 0){
        //     crocoddyl::FrameMotion Vref(fid, pinocchio::Motion (Eigen::Map<Eigen::VectorXd>(traj.data())));
        //     boost::shared_ptr<crocoddyl::CostModelAbstract> vel_tracking_cost = 
        //         boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, Vref);   

        //     rcost_arr_[sn].get()->addCost(cost_name, vel_tracking_cost, wt);
        // }
        // else{
            for (unsigned i = sn; i < en; ++i){
                crocoddyl::FrameMotion Mref(fid, pinocchio::Motion(traj.row(i)));
                boost::shared_ptr<crocoddyl::CostModelAbstract> vel_tracking_cost = 
                    boost::make_shared<crocoddyl::CostModelFrameVelocity>(state_, Mref);   

                rcost_arr_[i].get()->addCost(cost_name, vel_tracking_cost, wt);
            // }
        };
    };



}