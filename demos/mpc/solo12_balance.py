## This is a demo for 2-leg balance in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import copy
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_balance_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env
import os
import raisimpy as raisim
import subprocess

time.sleep(2)

## Motion
gait_params = balance

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
gait_time = gait_params.gait_period
dt = 5e-2

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
q0[10] = -0.8
q0[11] = 1.2
q0[12] = -2.2

q0[13] = 0.8
q0[14] = -1.2
q0[15] = 2.2

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0,0.0])
step_height = gait_params.step_ht

plan_freq = 0.002 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
robot = Solo12Env(gait_params.kp, gait_params.kd, q0, v0, False, False)

lag = int(update_time/sim_dt)

x0[10] = -0.8
x0[11] = 1.2
x0[12] = -2.2

x0[13] = 0.8
x0[14] = -1.2
x0[15] = 2.2

gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, gait_params, x0, plan_freq, q0, None)

q, v = robot.get_state()

# plotting
com_arr = []

for o in range(int(500*(plan_freq/sim_dt))):
    com_arr.append(robot.get_com_location())
    # this bit has to be put in shared memory
    if pln_ctr == 0:
        print("Time: ")
        print(sim_t)
        q, v = robot.get_state()
        # reseting origin (causes scaling issues I think otherwise)

        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), v_des, 0.0, gait_params.step_ht, contact_configuration)
        # if sim_t > 1.5:
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

    # if np.all((contact_configuration==0)):
    #     print("flight phase")
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    #robot.send_joint_command_tsid(sim_t, q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt
    step_t = (step_t + sim_dt)%gait_time

    time.sleep(0.001)
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")


