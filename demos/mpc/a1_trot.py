## This is a demo for trot motion in mpc
## Author : Paarth shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_a1.config import A1Config
from abstract_mpc_gait_gen import SoloMpcGaitGen
from a1_gait_params import trot, walk, bound, still, gallop

from py_biconvex_mpc.bullet_utils.a1_mpc_env import A1Env
import os


## Motion
gait_params = trot

## robot config and init
pin_robot = A1Config.buildRobotWrapper()
urdf_path = A1Config.urdf_path
gait_time = gait_params.gait_period
dt = 5e-2

n_eff = 4
q0 = np.array(A1Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([1.0,0.0,0.0])
step_height = gait_params.step_ht

plan_freq = 0.04 # sec
update_time = 0.008 # sec (time of lag)

sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
robot = A1Env(gait_params.kp, gait_params.kd, q0, v0, False, False)

lag = int(update_time/sim_dt)
gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, gait_params, x0, plan_freq, q0)

q, v = robot.get_state()

# plotting
com_arr = []

for o in range(int(500*(plan_freq/sim_dt))):
    com_arr.append(robot.get_com_location())
    if pln_ctr == 0:
        q, v = robot.get_state()
        # reseting origin (causes scaling issues I think otherwise)
        q[0:2] = 0
        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), v_des, gait_params.step_ht, contact_configuration)
        # if o >= 100:
        #     gg.plot_plan()
        gg.reset()
        pr_et = time.time()

    # update of plan
    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt)-1:
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    # control loop
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    #robot.send_joint_command_tsid(sim_t, q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt
    step_t = (step_t + sim_dt)%gait_time

    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")

# gg.plot(np.array(com_arr))

