#ifndef BICONVEX_MPC_ROBOT_MODEL_HPP
#define BICONVEX_MPC_ROBOT_MODEL_HPP

#include <pinocchio/fwd.hpp>  // always needs to be included first to avoid compilation errors

#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

namespace robot_model
{
    typedef Eigen::Matrix<double, 3, 4> Matrix34d;
    typedef Eigen::Matrix<bool, 4, 1> Vector4b;
/**
 * @brief RobotModel encapsulating a Pinocchio::Model
 *
 * This class computes and stores information for an abstract robot
 * from a Pinocchio::Model and a nominal configuration.
 *
 */
    class RobotModel
    {
    public:
        /**
         * @brief Construct a new Robot Model object from a URDF and semantic information
         *
         * @param urdf Can be either a path to a URDF file or a string containing an XML stream
         * @param q_nominal Nominal/rest/initial configuration
         * @param bodyName Name of the root / base link
         * @param lfFootName Name of the left front foot link (contact link)
         * @param lhFootName Name of the left hind foot link (contact link)
         * @param rfFootName Name of the right front foot link (contact link)
         * @param rhFootName Name of the right hind foot link (contact link)
         */
        RobotModel(const std::string& urdf, const Eigen::VectorXd& q_nominal, const std::string& bodyName, const std::string& lfFootName, const std::string& lhFootName, const std::string& rfFootName, const std::string& rhFootName);

        RobotModel(std::shared_ptr<pinocchio::Model> robot, std::shared_ptr<pinocchio::Data> data, const Eigen::VectorXd& q_nominal, const std::string& bodyName, const std::string& lfFootName, const std::string& lhFootName, const std::string& rfFootName, const std::string& rhFootName);

        void initialize(); //!< Initializes internal data structures - to be called from constructor.

        /**
         * @brief Get the composite inertia computed from q_nominal during instantiation.
         *
         * @return const Eigen::Matrix3d&
         */
        const Eigen::Matrix3d& get_composite_inertia() const { return I_comp_b_; }

        // Get base mass
        double get_body_mass() const { return robot_->inertias[1].mass(); }

        /**
         * @brief Get the end-effector offsets.
         * These are either set or computed automatically during instantiation.
         *
         * @return const Matrix34d&
         */
        const Matrix34d& get_eef_offsets() const { return eef_offsets_; }

        /**
         * @brief Get the end-effector positions in the world frame from the most recent forward kinematics update.
         * This method assumes that pinocchio::framesForwardKinematics has been called on the model/data before.
         * The foot order is:
         *
         * @return const Matrix34d&
         */
        const Matrix34d& get_eef_positions();

        /**
         * @brief Set the end-effector offsets (from the hip frame)
         *
         * @param lfOffset
         * @param lhOffset
         * @param rfOffset
         * @param rhOffset
         */
        void set_eef_offsets(const Eigen::Vector3d& lfOffset, const Eigen::Vector3d& lhOffset,
                             const Eigen::Vector3d& rfOffset, const Eigen::Vector3d& rhOffset);

        /**
         * @brief Get the current CoM of the base.
         *
         * @return const Eigen::Vector3d&
         */
        const Eigen::Vector3d& get_base_com() const { return data_->com[0]; }
        /**
         * @brief Get the current velocity of the CoM of the base
         *
         * @return const Eigen::Vector3d&
         */
        const Eigen::Vector3d& get_base_vcom() const { return data_->vcom[0]; }
        /**
         * @brief Get the current yaw of the base
         *
         * @return double
         */
        double get_base_yaw();

        /**
         * @brief Get the Rotation Matrix for the base in current position
         *
         * @return const Eigen::Matrix3d&
         */
        const Eigen::Matrix3d& get_base_rotation() const { return data_->oMf[body_frame_idx_].rotation(); }
        /**
         * @brief Updates the internal Pinocchio object (frames etc.) based on q and v
         *
         * @param q Configuration vector
         * @param v Velocity vector
         */

        void update(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
        /**
         * @brief Get the list of end-effector/leg frame identifiers in LF, RF, LH, RH order
         *
         * @return const Eigen::Vector4i
         */
        const Eigen::Vector4i& get_eef_frame_idx() const { return eef_frame_idx_; }
        /**
         * @brief Get the body frame index
         *
         * @return const int&
         */
        const int& get_body_frame_idx() const { return body_frame_idx_; }
        std::shared_ptr<pinocchio::Model> get_pinocchio_model() const { return robot_; }
        std::shared_ptr<pinocchio::Data> get_pinocchio_data() const { return data_; }
        const Eigen::VectorXd& get_q() const { return q_; }
        const Eigen::VectorXd& get_v() const { return v_; }

        void initialize();                                        //!< Initializes internal data structures - to be called from constructor.
        std::shared_ptr<pinocchio::Model> robot_;                 //!< Pinocchio robot model
        std::shared_ptr<pinocchio::Data> data_;                   //!< Pinocchio data

        Matrix34d eef_offsets_ = Matrix34d::Zero();               //!< Offsets from the hip frame to compute new footsteps (in base frame)
        Eigen::Vector4i eef_frame_idx_;                           //!< FrameIndex for the end-effectors
        int body_frame_idx_;                                      //!< FrameIndex for the body frame
        std::string body_name_;                                   //!< Name of the body/base link
        std::string lf_name_;                                     //!< Name of left-front foot link (FL)
        std::string rf_name_;                                     //!< Name of right-front foot link (FR)
        std::string lh_name_;                                     //!< Name of left-hind foot link (HL)
        std::string rh_name_;                                     //!< Name of right-hind foot link (HR)

        Eigen::VectorXd q_;
        Eigen::VectorXd v_;
    };
} //namespace robot_model

#endif //BICONVEX_MPC_ROBOT_MODEL_HPP
