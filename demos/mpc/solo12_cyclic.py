import time
import numpy as np

from paths.paths import Paths
from abstract_cyclic_gen import AbstractMpcGaitGen
from solo12_gait_params import trot

from environment_interface.raisim_interface import RaisimEnv
from controllers.robot_id_controller import InverseDynamicsController

# Load paths and robot_info
project_paths = Paths("solo12")

# Simulation Parameters
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

# Motion
v_des = np.array([0.5, 0.0, 0.0])
w_des = 0.0
plan_freq = 0.05  # sec #TODO: Should go inside motion
update_time = 0.0  # sec (time of lag) #TODO: Should go inside robot_info?
lag_counter = int(update_time / sim_dt)  # TODO: Can I remove this?
gait_params = trot
gait_generator = AbstractMpcGaitGen(project_paths.URDF_PATH, project_paths.ROBOT_INFO, plan_freq, None)
gait_generator.update_gait_params(gait_params, sim_t)

# Environment
robot_interface = RaisimEnv(project_paths.URDF_PATH, project_paths.ROBOT_INFO)
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

    if pln_ctr == 0:
        contact_configuration = robot_interface.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gait_generator.optimize(q, v, np.round(sim_t, 3), v_des, w_des)

        # Plot if necessary
        if sim_t >= plot_time:
            # gg.plot_plan(q, v)
            gait_generator.save_plan("trot")

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
        index = 0

    tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:19].copy(), \
                                         xs[index][19:].copy(), \
                                         us[index], f[index], contact_configuration)
    robot_interface.send_joint_command(tau)

    # time.sleep(0.001)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1) % (plan_freq / sim_dt))
    index += 1
