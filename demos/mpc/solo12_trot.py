## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import copy
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_mpc_gait_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance, bound_turn, trot_turn

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env
import os
import raisimpy as raisim
import subprocess

time.sleep(2)

## Motion
gait_params = bound

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
gait_time = gait_params.gait_period

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.4,0.0,0.0])
w_des = 0.0
step_height = gait_params.step_ht

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
robot = Solo12Env(gait_params.kp, gait_params.kd, q0, v0, False, False)

lag = int(update_time/sim_dt)
gg = SoloMpcGaitGen(pin_robot, urdf_path, gait_params, x0, plan_freq, q0, None)
q, v = robot.get_state()

plot_time = np.inf #Time to start plotting
ori_des = [0,0,0,1] #Desired Orientation

# plotting
com_arr = []

for o in range(int(500*(plan_freq/sim_dt))):
    com_arr.append(robot.get_com_location())

    # this bit has to be put in shared memory
    if pln_ctr == 0:
        q, v = robot.get_state()
        # reseting origin (causes scaling issues I think otherwise)
        q[0:2] = 0
        contact_configuration = robot.get_current_contacts()
        
        if w_des != 0:
            ori_des = q[3:7]
        
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), v_des, w_des, ori_des, gait_params.step_ht, contact_configuration)

        #Plot if necessary
        if sim_t > plot_time:
            gg.plot_plan()

        pr_et = time.time()
        # print("Full Time : ", pr_et - pr_st)
        # print("=========================")
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
    # step_t = (step_t + sim_dt)%gait_time
    step_t = (step_t + sim_dt)


    # if o > int(100*(plan_freq/sim_dt)):
    #     v_des[0] = 0.8
        
    time.sleep(0.001)
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")


