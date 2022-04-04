#include "ik/inverse_kinematics.hpp"
#include "action_model.cpp"

namespace ik{

    InverseKinematics::InverseKinematics(std::string rmodel_path, int n_col):
        n_col_(n_col) // wonder if this should come from the user?
    {

        pinocchio::urdf::buildModel(rmodel_path,pinocchio::JointModelFreeFlyer(), rmodel_) ;
        // temporaryily created 
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;
        m_ = pinocchio::computeTotalMass(rmodel_);
    
        state_ = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(rmodel_));

        // actuation_ = boost::ZZZmake_shared<crocoddyl::ActuationModelFloatingBase>(state_);
        actuation_ = boost::make_shared<crocoddyl::ActuationModelFull>(state_);

        for (unsigned i = 0; i < n_col_; i++){
            boost::shared_ptr<crocoddyl::CostModelSum> rcost_model =
                                        boost::make_shared<crocoddyl::CostModelSum>(state_);
            rcost_arr_.push_back(rcost_model);
        }

        // terminal cost model
        tcost_model_ = boost::make_shared<crocoddyl::CostModelSum>(state_);  
        rint_arr_ = std::vector< boost::shared_ptr<crocoddyl::ActionModelAbstract>>(n_col_);

        // 
        ik_com_opt_.resize(n_col+1, 3);
        ik_com_opt_.setZero();
        ik_mom_opt_.resize(n_col+1, 6);
        ik_mom_opt_.setZero();
    };

    void InverseKinematics::setup_costs(Eigen::VectorXd dt){

        for (unsigned i = 0; i < n_col_; i++){
            boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> running_DAM =
                        boost::make_shared<crocoddyl::DifferentialFwdKinematicsModelTpl<double>>(state_, actuation_, rcost_arr_[i]);
            rint_arr_[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(running_DAM, dt[i]);

        }

        boost::shared_ptr<crocoddyl::DifferentialFwdKinematicsModelTpl<double>> terminal_DAM =
                        boost::make_shared<crocoddyl::DifferentialFwdKinematicsModelTpl<double>>(state_, actuation_, tcost_model_);

        tint_model_ = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_DAM);

    };

    void InverseKinematics::optimize(const Eigen::VectorXd& x0){
        
        problem_ = boost::make_shared<crocoddyl::ShootingProblem>(x0, rint_arr_, tint_model_);
        ddp_ = boost::make_shared<crocoddyl::SolverDDP>(problem_);
        ddp_->solve();


        // wonder how effecient this ?
        for (unsigned i = 0; i < n_col_; i++){
            boost::shared_ptr<crocoddyl::CostModelSum> rcost_model =
                                        boost::make_shared<crocoddyl::CostModelSum>(state_);
            rcost_arr_[i] = rcost_model;
        }

        // terminal cost model
        tcost_model_ = boost::make_shared<crocoddyl::CostModelSum>(state_);  

    };

    void InverseKinematics::compute_optimal_com_and_mom(Eigen::MatrixXd &opt_com, Eigen::MatrixXd &opt_mom){
        // TODO: Change name to optimal_com_vel : to be used for kino-dyn iteration
        xs_ = ddp_->get_xs();
        for(unsigned i = 0; i < n_col_+1; ++i){
            pinocchio::computeCentroidalMomentum(rmodel_, rdata_, xs_[i].head(rmodel_.nq), xs_[i].tail(rmodel_.nv));
            opt_com.row(i) = rdata_.com[0];
            opt_mom.row(i) = rdata_.hg.toVector();
            opt_mom(i,0) /= m_; 
            opt_mom(i,1) /= m_;
            opt_mom(i,2) /= m_;
        }
    }

    Eigen::MatrixXd InverseKinematics::return_opt_com(){
        xs_ = ddp_->get_xs();
        for(unsigned i = 0; i < n_col_+1; ++i){
            pinocchio::computeCentroidalMomentum(rmodel_, rdata_, xs_[i].head(rmodel_.nq), xs_[i].tail(rmodel_.nv));
            ik_com_opt_.row(i) = rdata_.com[0];
        }
        return ik_com_opt_;
    }

    Eigen::MatrixXd InverseKinematics::return_opt_mom(){
        xs_ = ddp_->get_xs();
        for(unsigned i = 0; i < n_col_+1; ++i){
            pinocchio::computeCentroidalMomentum(rmodel_, rdata_, xs_[i].head(rmodel_.nq), xs_[i].tail(rmodel_.nv));
            ik_mom_opt_.row(i) = rdata_.hg.toVector();
        }
        return ik_mom_opt_;
    }
};