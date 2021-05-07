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

robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
st = 0.2
dt = 5e-2
state_wt = np.array([0.] * 3 + [1000.] * 3 + [5.0] * (robot.model.nv - 6) \
                        + [0.01] * 6 + [5.0] *(robot.model.nv - 6))

n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])

v_des = np.array([0.0, 0, 0])
sl_arr = v_des*st
t = 0.0
sh = 0.15

gg = SoloMpcGaitGen(robot, urdf_path, st, dt, state_wt, x0)

# while True:
n = 1

sim_t = 0.0
step_t = 0.0
sim_dt = 1e-3

robot = Solo12Env(2, 0.05)

while True:

    next_loc = np.array([[ 0.3946 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.3946 + sl_arr[0],  -0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],  -0.14695 + sl_arr[1], 0]])

    # this bit has to be put in shared memory
    if np.round(sim_t,2) % dt == 0:
        q, v = robot.get_state()
        st = time.time()
        xs, us, f = gg.optimize(q0, v0, step_t, n, next_loc, v_des, sh, 7e-3, 5e-4)
        gg.reset()
        et = time.time()
        print("time", et - st)
    
    # control loop
    q_des = xs[step_t][:robot.model.nq]
    v_des = xs[step_t][robot.model.nq:]
    robot.send_joint_command(q_des, v_des, us[step_t], f[step_t])
    sim_t += sim_dt
    step_t =  np.round((step_t + sim_dt),2)%st
    if step_t == 0:
        n += 1


# np.savez("../motion_planner/dat_file/ik", xs = xs)

# gg.plot()



