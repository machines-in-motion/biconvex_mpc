// This file contains regularization costs

#include "ik/inverse_kinematics.hpp"


namespace ik{

    void InverseKinematics::add_state_regularization_cost(int sn, int en, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights,
                                        Eigen::VectorXd x_reg, bool isTerminal)
    {

        if (!isTerminal){
            for (unsigned i = sn; i < en; ++i){
                boost::shared_ptr<crocoddyl::ActivationModelAbstract> state_activation =
                    boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(stateWeights);
                boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg =
                    boost::make_shared<crocoddyl::CostModelResidual>(state_, state_activation, 
                        boost::make_shared<crocoddyl::ResidualModelState>(state_, x_reg));
                        
                rcost_arr_[i].get()->addCost(cost_name, state_reg, wt);
            }
        }
        else{
            boost::shared_ptr<crocoddyl::ActivationModelAbstract> state_activation =
                boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(stateWeights);
            boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg =
                boost::make_shared<crocoddyl::CostModelResidual>(state_, state_activation, 
                    boost::make_shared<crocoddyl::ResidualModelState>(state_, x_reg));
            tcost_model_->addCost(cost_name, state_reg, wt);
        }
    };

    void InverseKinematics::add_state_regularization_cost_single(int time_step, double wt, 
                                        std::string cost_name, Eigen::VectorXd stateWeights,
                                        Eigen::VectorXd x_reg)
    {

        boost::shared_ptr<crocoddyl::ActivationModelAbstract> state_activation =
            boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(stateWeights);
        boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg =
            boost::make_shared<crocoddyl::CostModelResidual>(state_, state_activation, 
                boost::make_shared<crocoddyl::ResidualModelState>(state_, x_reg));
        
        rcost_arr_[time_step].get()->addCost(cost_name, state_reg, wt);  
    };

    void InverseKinematics::add_ctrl_regularization_cost_single(int time_step, double wt, 
                                                        std::string cost_name,  Eigen::VectorXd controlWeights, 
                                                        Eigen::VectorXd u_reg)
    {
        boost::shared_ptr<crocoddyl::ActivationModelAbstract> control_activation =
                    boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(controlWeights);
                
                boost::shared_ptr<crocoddyl::CostModelAbstract> ctrl_reg =
                     boost::make_shared<crocoddyl::CostModelResidual>(
                            state_, control_activation,
                            boost::make_shared<crocoddyl::ResidualModelControl>(state_));

                rcost_arr_[time_step].get()->addCost(cost_name,ctrl_reg, wt);
    
    }

    void InverseKinematics::add_ctrl_regularization_cost(int sn, int en, double wt, 
                                                        std::string cost_name,  Eigen::VectorXd controlWeights, 
                                                        Eigen::VectorXd u_reg, bool isTerminal)
    {
        if (!isTerminal){
            for (unsigned i = sn; i < en; ++i){
            
                boost::shared_ptr<crocoddyl::ActivationModelAbstract> control_activation =
                    boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(controlWeights);
                
                boost::shared_ptr<crocoddyl::CostModelAbstract> ctrl_reg =
                     boost::make_shared<crocoddyl::CostModelResidual>(
                            state_, control_activation,
                            boost::make_shared<crocoddyl::ResidualModelControl>(state_));

                rcost_arr_[i].get()->addCost(cost_name,ctrl_reg, wt);
            }
        }
        else{
            boost::shared_ptr<crocoddyl::ActivationModelAbstract> control_activation =
                boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(controlWeights);
            
            boost::shared_ptr<crocoddyl::CostModelAbstract> ctrl_reg =
                    boost::make_shared<crocoddyl::CostModelResidual>(
                        state_, control_activation,
                        boost::make_shared<crocoddyl::ResidualModelControl>(state_));
                        
            tcost_model_->addCost(cost_name,ctrl_reg, wt);
        }
    };

}