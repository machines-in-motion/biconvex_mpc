#include "inverse_kinematics.hpp"
#include "action_model.cpp"

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

        std::cout << rmodel_.nq << " " << rmodel_.nv << std::endl;

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
            boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> running_DAM =
                        boost::make_shared<crocoddyl::DifferentialFwdKinematicsModelTpl<double>>(state_, actuation_, rcost_arr_[i]);
            rint_arr_[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(running_DAM, dt_);

        }

        boost::shared_ptr<crocoddyl::DifferentialFwdKinematicsModelTpl<double>> terminal_DAM =
                        boost::make_shared<crocoddyl::DifferentialFwdKinematicsModelTpl<double>>(state_, actuation_, tcost_model_);

        tint_model_ = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_DAM);

    };

    void InverseKinematics::optimize(const Eigen::VectorXd& x0){
        
        problem_ = boost::make_shared<crocoddyl::ShootingProblem>(x0, rint_arr_, tint_model_);
        ddp_ = boost::make_shared<crocoddyl::SolverDDP>(problem_);
        ddp_->solve();

    };
}