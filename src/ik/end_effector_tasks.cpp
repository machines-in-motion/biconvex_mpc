// This file contains end effector tracking tasks

#include "inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_position_tracking_task(
                    pinocchio::FrameIndex fid, double st, double et, 
                    Eigen::MatrixXd traj, double wt, std::string cost_name){

        sn = st/dt_; 
        en = et/dt_;

        std::cout << "time " << sn << " " << en << std::endl;
        std::cout << traj.rows() << " " << traj.cols() << std::endl;
        std::cout << state_->get_nv() << std::endl;
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

}