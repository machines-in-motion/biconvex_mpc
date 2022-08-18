## This is a demo for trot motion in mpc
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_a1.a1wrapper import A1Robot, A1Config
from mpc.abstract_cyclic_gen import SoloMpcGaitGen
from motions.cyclic.a1_trot import trot

from envs.pybullet_env import PyBulletEnv
from controllers.robot_id_controller import InverseDynamicsController

## robot config and init
pin_robot = A1Config.buildRobotWrapper()
urdf_path = A1Config.urdf_path

n_eff = 4
q0 = np.array(A1Config.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

v_des = np.array([0.25, 0.0, 0.0])
w_des = 0.0

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

## Motion
gait_params = trot
lag = int(update_time/sim_dt)
gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)

gg.update_gait_params(gait_params, sim_t)

robot = PyBulletEnv(A1Robot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

plot_time = 0 #Time to start plotting

solve_times = []

for o in range(int(150*(plan_freq/sim_dt))):
    # this bit has to be put in shared memory
    q, v = robot.get_state()
    if pln_ctr == 0:
        contact_configuration = robot.get_current_contacts()

        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)

        # Plot if necessary
        # if sim_t >= plot_time:
        # gg.plot_plan(q, v)
        # gg.save_plan("trot")

        pr_et = time.time()
        solve_times.append(pr_et - pr_et)

    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt)-1:
        # Not the correct logic
        # lag = int((1/sim_dt)*(pr_et - pr_st))
        lag = 0
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy() \
                                         , us[index], f[index], contact_configuration)
    robot.send_joint_command(tau)

    # time.sleep(0.001)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

np.savez("./bound_" + str(gg.horizon))
print("done")


