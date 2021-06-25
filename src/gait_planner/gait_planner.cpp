#include "gait_planner/gait_planner.hpp"

namespace gait_planner
{
    QuadrupedGait::QuadrupedGait(double gait_period, Eigen::Vector4d stance_percent,
                                 Eigen::Vector4d phase_offset, double step_height)
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

        this->min_rel_height_ = 0;
    }

    Eigen::Vector4d QuadrupedGait::get_phi(double time_in)
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

    int QuadrupedGait::get_phase(double time_in, int foot_ID) {

        if (get_phi(time_in, foot_ID) <= stance_time_[foot_ID] ||
            abs(get_phi(time_in, foot_ID) - stance_time_[foot_ID]) < 1e-4)
        {
            phase_[foot_ID] = 1;  //Stance
        }
        else
        {
            phase_[foot_ID] = 0;  //Swing
        }
        return phase_[foot_ID];
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

    Eigen::Vector4d QuadrupedGait::get_percent_in_phase(double time_in)
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

    Eigen::MatrixXi QuadrupedGait::get_contact_phase_plan(Eigen::MatrixXi contact_phase_plan, double time_in, double dt)
    {
        for (unsigned int i = 0; i < contact_phase_plan.rows(); ++i) {
            contact_phase_plan.row(i) = get_phase(time_in + i*dt);
        }
        return contact_phase_plan;
    }

    double QuadrupedGait::get_percent_in_phase(double time, int foot_ID)
    {
        auto current_phi = get_phi(time, foot_ID);
        double phase_percent;
        //std::cout << current_phi << " " << foot_ID << std::endl;
        if (current_phi <= stance_time_[foot_ID])
        {
            //Stance
            phase_percent = current_phi / stance_time_[foot_ID];
        }
        else
        {
            //Swing
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


}  // namespace gait_planner