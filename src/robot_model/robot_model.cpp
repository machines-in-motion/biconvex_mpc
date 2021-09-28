#include "robot_model/robot_model.hpp"

#include <fstream>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace robot_model
{
    RobotModel::RobotModel(const std::string& urdf, const Eigen::VectorXd& q_nominal, const std::string& bodyName, const std::string& lfFootName, const std::string& lhFootName, const std::string& rfFootName, const std::string& rhFootName)
            : q_nominal_(q_nominal),
              body_name_(bodyName),
              lf_name_(lfFootName),
              rf_name_(rfFootName),
              lh_name_(lhFootName),
              rh_name_(rhFootName)
    {
        robot_ = std::make_shared<pinocchio::Model>();

        // Support for URDF from file and URDF from string:
        std::cout << "URDF is a file path - building model from file" << std::endl;
        pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(), *robot_.get());

        //Create data object
        data_ = std::make_shared<pinocchio::Data>(*robot_.get());

        //Initialize
        initialize();
    }

    RobotModel::RobotModel(std::shared_ptr<pinocchio::Model> robot, std::shared_ptr<pinocchio::Data> data, const Eigen::VectorXd& q_nominal, const std::string& bodyName, const std::string& lfFootName, const std::string& lhFootName, const std::string& rfFootName, const std::string& rhFootName)
            : robot_(robot),
              data_(data),
              q_nominal_(q_nominal),
              body_name_(bodyName),
              lf_name_(lfFootName),
              rf_name_(rfFootName),
              lh_name_(lhFootName),
              rh_name_(rhFootName)
    {
        std::cout << "Initializing RobotModel with:" << std::endl
                  << "  body = '" << body_name_ << "'" << std::endl
                  << "  LF   = '" << lf_name_ << "'" << std::endl
                  << "  LH   = '" << lh_name_ << "'" << std::endl
                  << "  RF   = '" << rf_name_ << "'" << std::endl
                  << "  RH   = '" << rh_name_ << "'" << std::endl;
        initialize();
    }

    void RobotModel::initialize()
    {
        //Assuming floating base Robot
        nq_ = robot_->nq;
        nv_ = robot_->nv;
        nu_ = robot_->nv - 6;

        q_.setZero(nq_);
        v_.setZero(nv_);

        pinocchio::framesForwardKinematics(*robot_.get(), *data_.get(), q_nominal_);

        // Get composite inertia matrix (i.e. 3x3 inertia of all links) at q_nominal configuration
        pinocchio::crba(*robot_.get(), *data_.get(), q_nominal_);
        I_comp_b_ = data_->Ycrb[1].inertia();

        // Check that all frames exists
        if (!robot_->existFrame(body_name_)) throw std::runtime_error("Could not find body frame associated with string '" + body_name_ + "'!");
        if (!robot_->existFrame(lf_name_)) throw std::runtime_error("Could not find LF frame associated with string '" + lf_name_ + "'!");
        if (!robot_->existFrame(rf_name_)) throw std::runtime_error("Could not find RF frame associated with string '" + rf_name_ + "'!");
        if (!robot_->existFrame(lh_name_)) throw std::runtime_error("Could not find LH frame associated with string '" + lh_name_ + "'!");
        if (!robot_->existFrame(rh_name_)) throw std::runtime_error("Could not find RH frame associated with string '" + rh_name_ + "'!");

        // Create frame_id vectors (for getting associated values more easily)
        auto com = get_base_com();
        for (std::size_t i = 0; i < robot_->frames.size(); ++i)
        {
            if (robot_->frames[i].name == body_name_)
            {
                body_frame_idx_ = i;
            }
            if (robot_->frames[i].name == lf_name_)
            {
                eef_frame_idx_[0] = i;
                eef_offsets_.col(0) = data_->oMf[i].translation() - com;
            }
            if (robot_->frames[i].name == lh_name_)
            {
                eef_frame_idx_[1] = i;
                eef_offsets_.col(1) = data_->oMf[i].translation() - com;
            }
            if (robot_->frames[i].name == rf_name_)
            {
                eef_frame_idx_[2] = i;
                eef_offsets_.col(2) = data_->oMf[i].translation() - com;
            }
            if (robot_->frames[i].name == rh_name_)
            {
                eef_frame_idx_[3] = i;
                eef_offsets_.col(3) = data_->oMf[i].translation() - com;
            }
        }
    }

    void RobotModel::set_eef_offsets(const Eigen::Vector3d& lfOffset, const Eigen::Vector3d& lhOffset,
                                     const Eigen::Vector3d& rfOffset, const Eigen::Vector3d& rhOffset)
    {
        eef_offsets_.col(0) = lfOffset;
        eef_offsets_.col(1) = lhOffset;
        eef_offsets_.col(2) = rfOffset;
        eef_offsets_.col(3) = rhOffset;
    }

    const Matrix34d& RobotModel::get_eef_positions()
    {
        for (int i = 0; i < 4; ++i)
        {
            eef_positions_.col(i) = data_->oMf.at(eef_frame_idx_[i]).translation();
        }
        return eef_positions_;
    }

    void RobotModel::update(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
    {
        if (q.size() != robot_->nq) throw std::runtime_error("Size of q is incorrect!");
        if (v.size() != robot_->nv) throw std::runtime_error("Size of v is incorrect!");

        q_ = q;
        v_ = v;
        pinocchio::normalize(*robot_.get(), q_);

        pinocchio::forwardKinematics(*robot_.get(), *data_.get(), q, v);
        pinocchio::centerOfMass(*robot_.get(), *data_.get(), q, v);
        pinocchio::computeJointJacobians(*robot_.get(), *data_.get(), q);
        pinocchio::updateFramePlacements(*robot_.get(), *data_.get());
        pinocchio::centroidalMomentum(*robot_.get(), *data_.get(), q, v)
    }

    double RobotModel::get_base_yaw()
    {
        auto w_R_b = data_->oMf[body_frame_idx_].rotation();
        return atan2(w_R_b(3, 2), w_R_b(3, 3));
    }
}  // namespace gait_planner