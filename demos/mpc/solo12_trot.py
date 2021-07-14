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
from abstract_mpc_gait_gen import SoloMpcGaitGen

from solo12_gait_params import trot, walk, bound

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import subprocess

# subprocess.Popen([r"/home/pshah/Applications/raisim/raisim_ws/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

time.sleep(2)

## Motion
gait_params = bound

## robot config and init

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
gait_time = gait_params.gait_period
dt = 5e-2

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
# q0[13:] = 2 * [0.0, 0.8, -1.6]

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0, 0])
step_height = gait_params.step_ht

plan_freq = 0.05 # sec
update_time = 0.02 # sec (time of lag)

gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, gait_params, x0, plan_freq, q0)

# while True:

sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
robot = Solo12Env(gait_params.kp, gait_params.kd, q0, v0, False, True)

lag = int(update_time/sim_dt)

q, v = robot.get_state()

# plotting
com_arr = []

for o in range(int(50*(gait_time/sim_dt))):
    com_arr.append(robot.get_com_location())
    # this bit has to be put in shared memory
    if pln_ctr == 0:
        q, v = robot.get_state()        
        # print(v[0:2])
        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), v_des, gait_params.step_ht, contact_configuration)

        # gg.plot_plan()
        gg.reset()
        pr_et = time.time()
        # print("time", pr_et - pr_st)
    
    # if np.round(step_t,3) == 0:
    #     print(gg.cnt_plan)
    #     print("-------------------------------")
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

    # if np.all((contact_configuration==0)):
    #     print("flight phase")
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt
    step_t = (step_t + sim_dt)%gait_time
    if o == 4*(gait_time/sim_dt):
        print("v_des updated")
        v_des = np.array([-0.5,0.0, 0])

    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")

gg.plot(np.array(com_arr))

