## Demo for robot doing a cartwheel 
## Author : Avadesh Meduri & Paarth Shah
## Date : 04/08/2021

import time
import numpy as np
import pinocchio as pin
from robot_properties_solo.config import Solo12Config
from blmc_controllers.robot_id_controller import InverseDynamicsController
from py_biconvex_mpc.bullet_utils.solo_mpc_env import AbstractEnv
from abstract_acyclic_gen import SoloAcyclicGen

# from motions.plan_cartwheel import plan
# from motions.rearing import plan
# from motions.plan_hifive import plan
# from motions.stand import plan
from motions.plan_jump import plan

pin_robot = Solo12Config.buildRobotWrapper()
rmodel = pin_robot.model
urdf = Solo12Config.urdf_path

q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

robot = AbstractEnv(q0, v0, False, False)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
plan_freq = 0.9 # sec
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = SoloAcyclicGen(pin_robot, urdf, plan_freq)
mg.update_motion_params(plan, sim_t)

time.sleep(2)

plot_time = 0.005

## Data plotting (must be moved into a class)
F_arr = []
f_des = []
for o in range(int(2*(plan_freq/sim_dt))):

    contact_configuration = robot.get_current_contacts()
    q, v = robot.get_state()
    robot_id_ctrl.set_gains(plan.kp, plan.kd)

    if pln_ctr == 0 or sim_t == 0:
        # if sim_t > max(plot_time, 0.001):
        #     mg.plot(q_arr, v_arr, F_arr, plot_force=True)
        #     assert False

        xs, us, f = mg.optimize(q, v, sim_t)
        xs = xs[lag:]
        us = us[lag:]
        f = f[lag:]
        index = 0
    

    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    tau = robot_id_ctrl.id_joint_torques(q, v, q_des, dq_des, us[index], f[index], contact_configuration)
    robot.send_joint_command(tau)
    
    F_arr.append(robot.get_ground_reaction_forces())
    f_des.append(f[index])
    time.sleep(0.005)

    sim_t += sim_dt
    sim_t = np.round(sim_t, 3)
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

from matplotlib import pyplot as plt
F_arr = np.asarray(F_arr)
f_des = np.asarray(f_des)
eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
fig, ax_f = plt.subplots(4, 1, sharex=True, sharey=True)
for n in range(4):
    # ax_f[n].plot(f_des[:,3*n], label = eff_names[n] + " Fx")
    # ax_f[n].plot(f_des[:,3*n+1], label = eff_names[n] + " Fy")
    ax_f[n].plot(f_des[:,3*n+2], label = eff_names[n] + " Fz")

    # ax_f[n].plot(F_arr[:,3*n], label = eff_names[n] + " Fx")
    # ax_f[n].plot(F_arr[:,3*n+1], label = eff_names[n] + " Fy")
    ax_f[n].plot(F_arr[:,3*n+2], label = "real " + eff_names[n] + " Fz")

    ax_f[n].grid()
    ax_f[n].legend()

plt.show()