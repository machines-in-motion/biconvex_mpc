#include "gait_planner/quadruped_gait.hpp"

namespace gait_planner
{
    QuadrupedGait::QuadrupedGait(double gait_period, const Eigen::Vector4d& stance_percent,
                                 const Eigen::Vector4d& phase_offset, double step_height)
            : gait_period_(gait_period)
    {
        // TODO: Cannot initialise all of these yet directly due to reorder (!)
        this->stance_percent_ = stance_percent;
        this->stance_time_ = gait_period_ * stance_percent_;
        this->swing_percent_ << 1.0 - stance_percent_[0], 1.0 - stance_percent_[1], 1.0 - stance_percent_[2], 1.0 - stance_percent_[3];
        this->swing_time_ << gait_period_ - stance_time_[0], gait_period_ - stance_time_[1],
                gait_period_ - stance_time_[2], gait_period_ - stance_time_[3];
        this->phase_offset_ = phase_offset;
        this->step_height_ = step_height;

        // Initialise internal members: assumes starting at "0" time
        (void)get_phi(0);
        (void)get_phase(0);

        // Calculate minimum relative height:
        this->min_rel_height_ = robot_model_->get_eef_positions().row(2).minCoeff();
    }

    const Eigen::Vector4d& QuadrupedGait::get_phi(double time_in)
    {
        phi_ << std::fmod(time_in + phase_offset_[0] * gait_period_, gait_period_),
                std::fmod(time_in + phase_offset_[1] * gait_period_, gait_period_),
                std::fmod(time_in + phase_offset_[2] * gait_period_, gait_period_),
                std::fmod(time_in + phase_offset_[3] * gait_period_, gait_period_);
        return phi_;
    }

    double QuadrupedGait::get_phi(double time_in, int foot_ID)
    {
        return std::fmod(time_in + phase_offset_[foot_ID] * gait_period_, gait_period_);
    }

    Eigen::Vector4i QuadrupedGait::get_phase(double time_in)
    {
        //there's probably a more elegant way to do this (binary modulo operator on the Eigen vectors...)
        for (int i = 0; i < 4; ++i)
        {
            if (get_phi(time_in, i) <= stance_time_[i])
            {
                phase_[i] = 1;  //Stance
            }
            else
            {
                phase_[i] = 0;  //Swing
            }
        }
        return phase_;
    }

    const Eigen::Vector4d& QuadrupedGait::get_percent_in_phase(double time_in)
    {
        auto current_phi = get_phi(time_in);
        for (int i = 0; i < 4; ++i)
        {
            if (current_phi[i] <= stance_time_[i])
            {
                //Stance
                phase_percent_[i] = current_phi[i] / stance_time_[i];  //Stance
            }
            else
            {
                //Swing
                phase_percent_[i] = (current_phi[i] - stance_time_[i]) / (gait_period_ - stance_time_[i]);
            }
        }
        return phase_percent_;
    }

    void QaudrupedGait::get_contact_phase_sequence(Eigen::MatrixXd& contact_phase_plan, double time_in, double dt)
    {
        for (unsigned int i = 0; i < contact_phase_plan.rows(); ++i) {
            contact_phase_plan.row(i) = get_phase(time_in + i*dt);
        }
    }

    double QuadrupedGait::get_percent_in_phase(double time, int foot_ID)
    {
        auto current_phi = get_phi(time, foot_ID);
        double phase_percent;
        if (current_phi <= stance_time_[foot_ID])
        {
            phase_percent = current_phi / stance_time_[foot_ID];
        }
        else
        {
            phase_percent = (current_phi - stance_time_[foot_ID]) / (gait_period_ - stance_time_[foot_ID]);
        }
        return phase_percent;
    }

    void QuadrupedGait::set_stance_percent(double lf_stance_percent, double lh_stance_percent, double rf_stance_percent,
                                           double rh_stance_percent)
    {
        stance_percent_ << lf_stance_percent, lh_stance_percent, rf_stance_percent, rh_stance_percent;
        stance_time_ = gait_period_ * stance_percent_;
        swing_time_ << gait_period_ - stance_percent_[0], gait_period_ - stance_percent_[1],
                gait_period_ - stance_percent_[2], gait_period_ - stance_percent_[3];
    }

    void QuadrupedGait::get_footsteps(std::vector<Eigen::MatrixXd>& foot_locs, double curr_time, double horizon, double dt,
                                      double input_Vx, double input_Vy, double input_W, double current_omega)
    {
        throw std::runtime_error("Needs to be fixed up for new API!");

        /*if (!(static_cast<int>(horizon / dt) == static_cast<int>(foot_locs.size())))
        {
            throw std::runtime_error("Foot Location Matrices and Horizon don't match");
        }
        auto com = robot_model_->get_base_com();
        Eigen::Vector3d ds = {input_Vx * dt, input_Vy * dt, 0};
        Eigen::Vector3d v_cmd = {input_Vx, input_Vy, 0};
        double dw = input_W / dt;
        //get necessary kinematic information from robot_model_
        auto init_eef_pos = robot_model_->get_eef_positions();
        auto vcom = robot_model_->get_base_vcom();
        auto eef_offset = robot_model_->get_eef_offsets();
        auto prevPhase = get_phase(curr_time);
        auto omega = current_omega;  //TODO get this from robot_model_
        auto k_vcom = 0.03;
        //Set matrix for keeping track of the next footstep
        Eigen::MatrixXd next_footstep(4, 3);
        next_footstep.setZero();
        //Get future footsteps
        for (int t = 0; t < int(horizon / dt); t++)
        {
            auto currentPhase = get_phase(curr_time + dt * t);
            if (t == 0)
            {
                for (int eef = 0; eef < 4; ++eef)
                {
                    //Set current footsteps based off current location
                    foot_locs[0].row(eef) << init_eef_pos.row(eef);
                    if (!currentPhase[eef])
                    {
                        //Calculate next footstep location if any feet are currently in swing (for interpolation)
                        //TODO add omega-term (v x w_cmd)
                        next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                        next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                        next_footstep(eef, 2) = 0.0;
                    }
                }
            }
            else
            {
                com += ds;
                omega += dw;
                for (int eef = 0; eef < 4; ++eef)
                {
                    // if stance
                    if (currentPhase[eef])
                    {
                        if (prevPhase[eef])
                        {
                            foot_locs[t].row(eef) << foot_locs[t - 1].row(eef);
                        }
                        else
                        {
                            //I might want to see how stable my gait is calculating this here: (likely more stable)
                            //                        next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                            //                        next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                            //                        next_footstep(eef, 2) = min_rel_height_;
                            foot_locs[t].row(eef) << next_footstep.row(eef);
                        }
                    }
                    else
                    {
                        //Calculate next touchdown location for interpolation
                        //TODO: Add omega term (v x w_cmd)
                        next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                        next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                        next_footstep(eef, 2) = min_rel_height_;
                        //Calculate swing foot trajectory
                        double percentSwing = get_percent_in_phase(curr_time + dt * t, eef);
                        foot_locs[t].row(eef) << percentSwing * (next_footstep(eef, 0) - foot_locs[t - 1](eef, 0)) + foot_locs[t - 1](eef, 0),
                            percentSwing * (next_footstep(eef, 1) - foot_locs[t - 1](eef, 1)) + foot_locs[t - 1](eef, 1),
                            step_height_ * sin(percentSwing * M_PI) + min_rel_height_;
                    }
                }
            }
            prevPhase = currentPhase;
        }*/
    }

    void QuadrupedGait::get_footsteps_classical(std::vector<Eigen::MatrixXd>& foot_locs, double curr_time, double horizon, double dt,
                                                double input_Vx, double input_Vy, double input_W, double current_omega)
    {
        throw std::runtime_error("Needs to be fixed up for new API!");

        /*if (!(static_cast<int>(horizon / dt) == static_cast<int>(foot_locs.size())))
        {
            throw std::runtime_error("Foot Location Matrices and Horizon don't match");
        }
        auto com = robot_model_->get_base_com();
        Eigen::Vector3d ds = {input_Vx * dt, input_Vy * dt, 0};
        Eigen::Vector3d v_cmd = {input_Vx, input_Vy, 0};
        double dw = input_W / dt;
        //get necessary kinematic information from robot_model_
        auto init_eef_pos = robot_model_->get_eef_positions();
        auto vcom = robot_model_->get_base_vcom();
        auto eef_offset = robot_model_->get_eef_offsets();
        auto prevPhase = get_phase(curr_time);
        auto omega = current_omega;  //TODO get this from robot_model_
        auto k_vcom = 0.03;
        //Set matrix for keeping track of the next footstep
        Eigen::MatrixXd next_footstep(4, 3);
        next_footstep = init_eef_pos;
        Eigen::MatrixXd prev_footstep(4, 3);
        prev_footstep.setZero();
        //Get future footsteps
        for (int t = 0; t < int(horizon / dt); t++)
        {
            auto currentPhase = get_phase(curr_time + dt * t);
            if (t == 0)
            {
                for (int eef = 0; eef < 4; ++eef)
                {
                    //Set current footsteps based off current location
                    foot_locs[0].row(eef) << init_eef_pos.row(eef);
                    if (!currentPhase[eef])
                    {
                        //Calculate next footstep location if any feet are currently in swing (for interpolation)
                        //TODO add omega-term (v x w_cmd)
                        next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                        next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                        next_footstep(eef, 2) = 0.0;
                    }
                }
            }
            else
            {
                com += ds;
                omega += dw;
                for (int eef = 0; eef < 4; ++eef)
                {
                    // if stance
                    if (currentPhase[eef])
                    {
                        if (prevPhase[eef])
                        {
                            foot_locs[t].row(eef) << foot_locs[t - 1].row(eef);
                        }
                        else
                        {
                            //I might want to see how stable my gait is calculating this here: (likely more stable)
                            //                        next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                            //                        next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                            //                        next_footstep(eef, 2) = min_rel_height_;
                            foot_locs[t].row(eef) << next_footstep.row(eef);
                        }
                    }
                    else
                    {
                        if (prevPhase[eef])
                        {
                            prev_footstep.row(eef) = next_footstep.row(eef);
                            next_footstep.row(eef) = com.transpose() + eef_offset.row(eef);
                            next_footstep.row(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                            next_footstep(eef, 2) = min_rel_height_;
                        }
                        //Calculate swing foot trajectory
                        double percentSwing = get_percent_in_phase(curr_time + dt * t, eef);
                        foot_locs[t].row(eef) << percentSwing * (next_footstep(eef, 0) - prev_footstep(eef, 0)) + prev_footstep(eef, 0),
                            percentSwing * (next_footstep(eef, 1) - prev_footstep(eef, 1)) + prev_footstep(eef, 1),
                            step_height_ * sin(percentSwing * M_PI) + min_rel_height_;
                    }
                }
            }
            prevPhase = currentPhase;
        }*/
    }

    void QuadrupedGait::get_footsteps_classical_post_swing(ContactSequence& foot_locs, double curr_time, double horizon, double dt,
                                                           double input_Vx, double input_Vy, double input_W)
    {
        std::size_t N = static_cast<std::size_t>(horizon / dt);
        if (N != foot_locs.size())
        {
            throw std::runtime_error("Foot Location Matrices and Horizon don't match");
        }

        Eigen::Vector3d com = robot_model_->get_base_com();         // NB: We are adding to this, so need a local copy.
        Eigen::Matrix3d w_R_b = robot_model_->get_base_rotation();  // NB: same as above
        const Eigen::Vector3d ds(input_Vx * dt, input_Vy * dt, 0);
        const Eigen::Vector3d v_cmd(input_Vx, input_Vy, 0);
        const Eigen::Vector3d dw(0.0, 0.0, input_W * dt);
        Eigen::Matrix3d exp_w = pinocchio::exp3(dw);

        //Rotation Matrices for post-swing trajectories (i.e. input_W*stance_time_[footID]) to project further in future
        //TODO: Test whether this is necessary
        Matrix34d dw_swing;
        dw_swing.setZero();
        std::vector exp_w_swing = std::vector<Eigen::Matrix3d>(4);

        // Get necessary kinematic information from robot_model_
        const Matrix34d& init_eef_pos = robot_model_->get_eef_positions();
        const Eigen::Vector3d& vcom = robot_model_->get_base_vcom();
        const Matrix34d& eef_offset = robot_model_->get_eef_offsets();

        //    auto omega = current_omega;  // TODO: get this from robot_model_
        const double k_vcom = 0.03;  //<! Feedback term for the CoM velocity

        // Set matrix for keeping track of the next footstep
        Matrix34d next_footstep = init_eef_pos;
        Matrix34d prev_footstep = Matrix34d::Zero();

        // Get future footsteps
        for (std::size_t t = 0; t < N; ++t)
        {
            foot_locs[t].in_contact = get_phase(curr_time + dt * t);

            if (t == 0)
            {
                for (int eef = 0; eef < 4; ++eef)
                {
                    // Set current footsteps based off current location
                    foot_locs[0].footsteps.col(eef) = init_eef_pos.col(eef);

                    dw_swing.col(eef) << 0.0, 0.0, input_W * stance_time_[eef];
                    exp_w_swing[eef] = pinocchio::exp3(dw_swing.col(eef));
                    // Calculate next footstep location if any feet are currently in swing (for interpolation)
                    if (foot_locs[t].in_contact(eef) == 0)  // 0 = swing
                    {
                        // TODO: add omega-term (v x w_cmd)
                        next_footstep.col(eef) = com + w_R_b * eef_offset.col(eef);
                        if (contact_post_swing_)
                        {
                            next_footstep.col(eef) += swing_time_[eef] * v_cmd;
                        }
                        next_footstep.col(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                        next_footstep(2, eef) = 0.0;  // z = 0.0
                    }
                }
            }
            else
            {
                com += ds;
                w_R_b *= exp_w;
                for (int eef = 0; eef < 4; ++eef)
                {
                    // Stance / in-contact
                    if (foot_locs[t].in_contact(eef) == 1)
                    {
                        // If the previous time-step was also in stance, copy location from previous time-step
                        if (foot_locs[t - 1].in_contact(eef) == 1)
                        {
                            foot_locs[t].footsteps.col(eef) = foot_locs[t - 1].footsteps.col(eef);
                        }
                            // Else copy from calculated contact location, but now in contact
                        else
                        {
                            foot_locs[t].footsteps.col(eef) = next_footstep.col(eef);
                        }
                    }
                        // Swing
                    else
                    {
                        // If previous time-step was in-contact
                        if (foot_locs[t - 1].in_contact(eef) == 1)
                        {
                            prev_footstep.col(eef) = next_footstep.col(eef);
                            next_footstep.col(eef) = com + w_R_b * eef_offset.col(eef);
                            if (contact_post_swing_)
                            {
                                next_footstep.col(eef) += swing_time_[eef] * v_cmd;
                            }
                            next_footstep.col(eef) += stance_time_[eef] * vcom / 2 + k_vcom * (vcom - v_cmd);
                            next_footstep(2, eef) = min_rel_height_;
                        }

                        // Calculate swing foot trajectory
                        double percentSwing = get_percent_in_phase(curr_time + dt * t, eef);
                        foot_locs[t].footsteps.col(eef).x() = percentSwing * (next_footstep(0, eef) - prev_footstep(0, eef)) + prev_footstep(0, eef);
                        foot_locs[t].footsteps.col(eef).y() = percentSwing * (next_footstep(1, eef) - prev_footstep(1, eef)) + prev_footstep(1, eef);
                        foot_locs[t].footsteps.col(eef).z() = step_height_ * std::sin(percentSwing * M_PI) + min_rel_height_;
                    }
                }
            }
        }
    }
}  // namespace gait_planner