#ifndef BICONVEX_MPC_GAIT_PLANNER_HPP
#define BICONVEX_MPC_GAIT_PLANNER_HPP

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>

namespace gait_planner
{
    //typedef Eigen::Matrix<double, 3, 4> Matrix34d;  // We store each foot in a separate column

    class QuadrupedGait
    {
    public:
        QuadrupedGait(double gait_period, Eigen::VectorXd stance_percent,
                      Eigen::VectorXd phase_offset, double step_height);

        // Get how far into the full gait cycle you should be in given the offset + time
        Eigen::VectorXd get_phi(double time_in);

        // Get how far into the full gait cycle you should given the offset + time for a specific foot
        double get_phi(double time, int foot_ID);

        // Get which phase you are currently in
        // 0 indicates swing/no contact, 1 indicates stance/contact
        Eigen::VectorXi get_phase(double time_in);

        // Get which phase you are currently in for a specific foot_ID
        // 0 indicates swing, 1 indicates stance
        int get_phase(double time, int foot_ID);

        //Get how far into the gait phase (swing or stance) the robot is in for all 4 end-effectors
        Eigen::VectorXd get_percent_in_phase(double time_in);

        //Get how far into the gait phase (swing or stance) the robot is in for a specific end-effector
        double get_percent_in_phase(double time, int foot_ID);

        //Pass in a pre-made matrix with the horizon / length you want, and compute
        // the entire contact sequence for a fixed dt
        Eigen::MatrixXi get_contact_phase_plan(Eigen::MatrixXi contact_phase_plan, double time_in, double dt);

        void set_step_height(double step_height) { step_height_ = step_height; };
        void set_stance_percent(double lf_stance_percent, double lh_stance_percent, double rf_stance_percent,
                                double rh_stance_percent);
    private:
        int n_eff; // number of end effector
        Eigen::VectorXd stance_percent_;
        Eigen::VectorXd swing_percent_;
        Eigen::VectorXd stance_time_;
        Eigen::VectorXd swing_time_;
        Eigen::VectorXd phase_offset_;  //Initial phase offset, used to determine gait pattern. Must be between 0-1.

        Eigen::VectorXi phase_;
        Eigen::VectorXd phi_;
        Eigen::VectorXd phase_percent_;

        bool contact_post_swing_ = true;

        double gait_period_;

        double step_height_ = 0.1;
        double min_rel_height_ = 0.0;  //minimum relative height from which the step_height should step from (i.e. for weird contact issues in a simulator)
        enum interp_Type
        {
            sine,
            linear,
            spline
        };
    };
}  // namespace gait_planner

#endif //BICONVEX_MPC_GAIT_PLANNER_HPP
