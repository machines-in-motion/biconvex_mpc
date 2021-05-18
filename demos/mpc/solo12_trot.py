## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config
from mpc_gait_gen import SoloMpcGaitGen

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env


import threading

## robot configf and init

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
st = 0.2
dt = 5e-2
state_wt = np.array([0.] * 3 + [1000.] * 3 + [5.0] * (pin_robot.model.nv - 6) \
                        + [0.01] * 6 + [5.0] *(pin_robot.model.nv - 6))

n_eff = 4
m = pin.computeTotalMass(pin_robot.model)
q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0, 0, 0])
sl_arr = v_des*st
t = 0.0
sh = 0.15
plan_freq = 0.05 # sec

gg = SoloMpcGaitGen(pin_robot, urdf_path, st, dt, state_wt, x0, plan_freq, gait = 0)

# while True:
n = 1

sim_t = 0.0
step_t = 0
sim_dt = 1e-3
index = 0
robot = Solo12Env(2.5, 0.05)

for o in range(int(3*(st/sim_dt))):

    next_loc = np.array([[ 0.3946 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.3946 + sl_arr[0],  -0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],  -0.14695 + sl_arr[1], 0]])

    # this bit has to be put in shared memory
    if index == 0:
        # print(index, step_t)
        q, v = robot.get_state()
        pr_st = time.time()
        xs, us, f = gg.optimize(q, v, np.round(step_t,3), n, next_loc, v_des, sh, 7e-3, 5e-4)
        gg.reset()
        pr_et = time.time()
        # print("time", pr_et - pr_st)
    
    # control loop
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index])
    sim_t += sim_dt
    step_t = (step_t + sim_dt)%st
    index = int((index + 1)%(plan_freq/sim_dt))

    if np.round(step_t,3) == 0:
        n += 1


gg.plot()

# np.savez("../motion_planner/dat_file/ik", xs = xs)

# gg.plot()



