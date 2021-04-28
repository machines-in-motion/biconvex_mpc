// This file contains regularization costs

#include "inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_state_regularization_cost(double st, double et, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights,
                                        Eigen::VectorXd x_reg)
    {
        sn = st/dt_; 
        en = et/dt_;                            
        for (unsigned i = sn; i < en; ++i){
            boost::shared_ptr<crocoddyl::ActivationModelAbstract> state_activation =
                boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(stateWeights);
            boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg =
                boost::make_shared<crocoddyl::CostModelState>(state_, state_activation, x_reg);
            
            rcost_arr_[i].get()->addCost(cost_name, state_reg, wt);
        }
    };

    void InverseKinematics::add_ctrl_regularization_cost(double st, double et, double wt, 
                                            std::string cost_name)
    {
        sn = st/dt_; 
        en = et/dt_;                            
        for (unsigned i = sn; i < en; ++i){
            boost::shared_ptr<crocoddyl::CostModelAbstract> ctrl_reg =
                boost::make_shared<crocoddyl::CostModelControl>(state_);
            rcost_arr_[i].get()->addCost(cost_name,ctrl_reg, wt);
        }
    };

}