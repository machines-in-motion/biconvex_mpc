## Demo for robot doing a cartwheel 
## Author : Avadesh Meduri
## Date : 04/08/2021

import time
import numpy as np
import pinocchio as pin
import crocoddyl
from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config

import raisimpy as raisim
import subprocess
from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

from mpc_cartwheel import generate_plan


robot = Solo12Config.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

pin_robot = Solo12Config.buildRobotWrapper()
urdf = Solo12Config.urdf_path


q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

x_reg2 = x0.copy()
x_reg2[3:7] = [0,1,0,0]

x_reg2[7:13] = 2 * [0.0, -np.pi + 0.8, -1.6]
x_reg2[13:19] = 2 * [0.0, -np.pi - 0.8, 1.6]

subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

robot = Solo12Env(7.5, 0.2, q0, v0, False, False)
time.sleep(2)
sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
plan_freq = 0.5 #0.05 # sec
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)


for o in range(int(500*(plan_freq/sim_dt))):

    contact_configuration = robot.get_current_contacts()
    q, v = robot.get_state()

    if pln_ctr == 0:
        xs, us, f = generate_plan(q, v, sim_t)
        xs = xs[lag:]
        us = us[lag:]
        f = f[lag:]
        index = 0

    # if index < len(xs) - 1:
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    # else:
    #     q_des = x_reg2[0:len(q0)]
    #     dq_des = v0
    #     index = min(index, len(xs) - 1)
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)

    time.sleep(0.01)

    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1



