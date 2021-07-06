## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config
# from mpc_gait_gen import SoloMpcGaitGen
from abstract_mpc_gait_gen import SoloMpcGaitGen

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import subprocess

# subprocess.Popen([r"/home/pshah/Applications/raisim/raisim_ws/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

time.sleep(2)

## robot config and init

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
gait_time = 0.5
dt = 5e-2
state_wt = np.array([0.] * 3 + [10] * 3 + [5.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [0.01] * 3 + [10.0] *(pin_robot.model.nv - 6))

n_eff = 4
m = pin.computeTotalMass(pin_robot.model)
q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0, 0.0, 0])
sl_arr = v_des*gait_time*0.5
t = 0.0
step_height = 0.07


plan_freq = 0.05 # sec
update_time = 0.02 # sec (time of lag)

# gg = SoloMpcGaitGen(pin_robot, urdf_path, st, dt, state_wt, x0, plan_freq, gait = 0)
gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, state_wt, x0, plan_freq, q0)

# while True:
# n = 1

sim_t = 0.0
step_t = 0
sim_dt = .001
index = 0
pln_ctr = 0
robot = Solo12Env(2.5, 0.01, q0, v0, False, True)

lag = int(update_time/sim_dt)

q, v = robot.get_state()

# plotting
com_arr = []

for o in range(int(5*(gait_time/sim_dt))):
    com_arr.append(robot.get_com_location())
    # this bit has to be put in shared memory
    if pln_ctr == 0:
        q, v = robot.get_state()
        hip_loc = robot.get_hip_locations()
        next_loc = np.array([[ hip_loc[0][0] + sl_arr[0],  hip_loc[0][1] + sl_arr[1], 0],
                            [ hip_loc[1][0] + sl_arr[0],  hip_loc[1][1] + sl_arr[1], 0],
                            [ hip_loc[2][0] + sl_arr[0],  hip_loc[2][1] + sl_arr[1], 0],
                            [ hip_loc[3][0] + sl_arr[0],  hip_loc[3][1] + sl_arr[1], 0]])

        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        # xs_plan, us_plan, f_plan = gg.optimize(q, v, round(sim_t*1000)/1000, next_loc, v_des, step_height, 5e-4, 9e-6, contact_configuration)
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), next_loc, v_des, step_height, 5e-3, 7e-4, contact_configuration)

        # gg.plot_plan()
        gg.reset()
        pr_et = time.time()
        # print("time", pr_et - pr_st)
    
    # update of plan
    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt):
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt):
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    # control loop

    if np.all((contact_configuration==0)):
        print("flight phase")
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt

    step_t = (step_t + sim_dt)%gait_time
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1
    # if np.round(step_t,3) == 0:
    #     n += 1

print("done")

gg.plot(np.array(com_arr))

