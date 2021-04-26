#include "inverse_kinematics.hpp"

namespace ik{

    InverseKinematics::InverseKinematics(std::string rmodel_path, double dt, double T):
        T_(T), dt_(dt), n_col_(int (T/dt))
    {

        pinocchio::urdf::buildModel(rmodel_path,rmodel_);
        // temporaryily created 
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;
        state_ = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(rmodel_));

        actuation_ = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

        for (unsigned i = 0; i < n_col_; i++){
            boost::shared_ptr<crocoddyl::CostModelSum> rcost_model =
                                        boost::make_shared<crocoddyl::CostModelSum>(state_);
            rcost_arr_.push_back(rcost_model);
        }

        // terminal cost model
        tcost_model_ = boost::make_shared<crocoddyl::CostModelSum>(state_);  
        rint_arr_ = std::vector< boost::shared_ptr<crocoddyl::ActionModelAbstract>>(n_col_);
    };

    void InverseKinematics::setup_costs(){

        for (unsigned i = 0; i < n_col_; i++){
//             boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> running_DAM =
//                         boost::make_shared<crocoddyl::DifferentialFwdKinematicsModel>(state_, actuation_, rcost_arr_[i]);
            
            //// Changing to line below does not throw seg fault
            
             boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> running_DAM =
                         boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, rcost_arr_[i]);
            
            std::cout << "hello" << std::endl;

            // rint_arr_[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(running_DAM, dt_);
            // std::cout << "hello" << std::endl;

        }

        // boost::shared_ptr<crocoddyl::DifferentialFwdKinematicsModel> terminal_DAM =
        //                 boost::make_shared<crocoddyl::DifferentialFwdKinematicsModel>(state_, actuation_, tcost_model_);

        // tint_model_ = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_DAM);

    };

    // void InverseKinematics::optimize(const Eigen::VectorXd& x0){
        
    //     problem_ = boost::make_shared<crocoddyl::ShootingProblem>(x0, rint_arr_, tint_model_);
    //     ddp_ = boost::make_shared<crocoddyl::SolverDDP>(problem_);
    //     ddp_->solve();

    // };
}