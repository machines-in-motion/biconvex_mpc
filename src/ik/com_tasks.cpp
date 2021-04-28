// This file contains com tracking tasks

#include "inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_com_position_tracking_task(double st, double et, Eigen::MatrixXd traj, 
                                                double wt, std::string cost_name, bool isTerminal)
        {
            sn = st/dt_; 
            en = et/dt_;

            if (!isTerminal){
                for (unsigned i = sn; i < en; ++i){
                    boost::shared_ptr<crocoddyl::CostModelAbstract> com_track =
                            boost::make_shared<crocoddyl::CostModelCoMPosition>(state_, traj.row(i));
                    rcost_arr_[i].get()->addCost(cost_name, com_track, wt);
                }
            }
            else{
                boost::shared_ptr<crocoddyl::CostModelAbstract> com_track =
                            boost::make_shared<crocoddyl::CostModelCoMPosition>(state_, traj);
                tcost_model_->addCost(cost_name, com_track, wt);
            }
        };

        void InverseKinematics::add_centroidal_momentum_tracking_task(double st, double et, Eigen::MatrixXd traj, 
                                                double wt, std::string cost_name, bool isTerminal)
        {
            sn = st/dt_; 
            en = et/dt_;

            if (!isTerminal){
                for (unsigned i = sn; i < en; ++i){
                    boost::shared_ptr<crocoddyl::CostModelAbstract> mom_track =
                            boost::make_shared<crocoddyl::CostModelCentroidalMomentum>(state_, traj.row(i));
                    rcost_arr_[i].get()->addCost(cost_name, mom_track, wt);
                }
            }
            else{
                boost::shared_ptr<crocoddyl::CostModelAbstract> mom_track =
                            boost::make_shared<crocoddyl::CostModelCentroidalMomentum>(state_, traj);
                tcost_model_->addCost(cost_name, mom_track, wt);
            }
        };

    



}