import numpy as np
import time

from paths.paths import Paths
from abstract_cyclic_gen import AbstractMpcGaitGen
from a1_gait_params import still, trot

from environment_interface.raisim_interface import RaisimEnv
from controllers.robot_id_controller import InverseDynamicsController

# Load paths and robot_info
project_paths = Paths("a1")

# Simulation Parameters
sim_t = 0.0
sim_dt = .001
ctrlr_index = 0
pln_ctr = 0

# Environment
robot_interface = RaisimEnv(project_paths.URDF_PATH, project_paths.ROBOT_INFO, sim_dt)

# Set Motion Parameters
v_des = np.array([0.8, 0.0, 0.0])
w_des = 0.0
plan_freq = 0.05  # sec #TODO: Should go inside motion
update_time = 0.0  # sec (time of lag) #TODO: Should go inside robot_info?
lag_counter = int(update_time / sim_dt)  # TODO: Can I remove this?

# Choose Motion
gait_params = trot
gait_generator = AbstractMpcGaitGen(project_paths.URDF_PATH, project_paths.ROBOT_INFO, plan_freq, None)
gait_generator.update_gait_params(gait_params, sim_t)

# Controller
robot_id_ctrl = InverseDynamicsController(project_paths.URDF_PATH, project_paths.ROBOT_INFO['eef_names'])
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

# Plotting
plot_time = np.inf

for o in range(int(500 * (plan_freq / sim_dt))):
    q, v = robot_interface.get_state()

    # if o == int(50*(plan_freq/sim_dt)):
    #     gait_params = trot
    #     gg.update_gait_params(gait_params, sim_t)
    #     robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

    # Update MPC Plan
    if pln_ctr == 0:
        contact_configuration = robot_interface.get_current_contacts()
        xs_plan, us_plan, f_plan = gait_generator.optimize(q, v, np.round(sim_t, 3), v_des, w_des)

        # Plot if necessary
        if sim_t >= plot_time:
            gait_generator.plot_plan(q, v)
            #gait_generator.save_plan("trot")

    # first loop assume that trajectory is planned
    if o < int(plan_freq / sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag_counter and o > int(plan_freq / sim_dt) - 1:
        # Not the correct logic
        # lag = int((1/sim_dt)*(pr_et - pr_st))
        lag_counter = 0
        xs = xs_plan[lag_counter:]
        us = us_plan[lag_counter:]
        f = f_plan[lag_counter:]
        ctrlr_index = 0

    #Calculate Torque
    tau = robot_id_ctrl.id_joint_torques(q, v, xs[ctrlr_index][:19].copy(), \
                                         xs[ctrlr_index][19:].copy(), \
                                         us[ctrlr_index], f[ctrlr_index], contact_configuration)

    #Send Torque to Robot
    robot_interface.send_joint_command(tau)

    #Increase Counters
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1) % (plan_freq / sim_dt))
    ctrlr_index += 1
