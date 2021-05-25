## This is demo for the inverse kinematics C++ version
## Author : Avadesh Meduri
## Date : 22/04/2021

import time
import numpy as np
from inverse_kinematics_cpp import InverseKinematics
import pinocchio as pin
from py_biconvex_mpc.ik_utils.abstract_gait_generator import AbstractGaitGenerator
from crocoddyl.utils.biped import SimpleBipedGaitProblem, plotSolution

urdf_path = "/home/pshah/Applications/atlas_urdf/robot.urdf"

dt = 5e-2
T = 1

pinModel = pin.buildModelFromUrdf("/home/pshah/Applications/atlas_urdf/robot.urdf", pin.JointModelFreeFlyer())
pinData = pinModel.createData()
q0 = np.array([ 0.0, 0.0, 0.95, #base
                1.0, 0.0, 0.0, 0.0, #base quaternion
                0.0, #hip yaw
                0.0487, #hip forward/backward
                0.0, #hip tilt
                0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, #left arm
                0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, #right arm
                0.0, 0.0, 0.0, #left hip abductors
                0.0, 0.0, 0.0, #left knee, ankle tilt fwd, ankle tilt side
                0.0, 0.0, 0.0, #right hip abductors
                0.0, 0.0, 0.0]) #right knee, right ankle tilt fwd, right ankle tilt side;
v = np.zeros(pinModel.nv)
x0 = np.concatenate([q0, np.zeros(pinModel.nv)])

stateWeights = np.array([0.] * 3 + [500.] * 3 + [0.01] * (pinModel.nv - 6) \
                        + [10.] * 6 + [5.0] *(pinModel.nv - 6))

# print(robot.model.nq, robot.model.nv)

des_pos_left = np.tile(np.array([0.0,   0.1115,  0]), (int(T/dt),1))
des_pos_right = np.tile(np.array([0.0,   -0.1115,  0]), (int(T/dt),1))

des_vel_left = np.tile(np.array([0.,   0,  0]), (int(T/dt),1))
des_vel_right = np.tile(np.array([0.,   -0,  0]), (int(T/dt),1))

des_com_pos = np.tile(np.array([0.,   0,  1.2]), (int(T/dt),1))
des_com_pos[:,0] = 0.1*np.linspace(0, len(des_com_pos), len(des_com_pos))

des_mom = np.tile(np.array([0.,   0,  0.0, 0, 0, 0]), (int(T/dt),1))
des_mom[:,0] = 0.1

gg = AbstractGaitGenerator(pinModel, pinData, urdf_path, T, dt)
gg.create_swing_foot_task(des_pos_left[0], des_pos_left[0] + [0.2, 0, 0], 0, 0.4, 0.1, "l_foot", "L_step", 1e3)
gg.create_contact_task(des_pos_left[0], 0, T, "r_foot", "R_foot", 1e5)
gg.create_contact_task(des_pos_right[0], 0, T, "l_foot", "L_foot", 1e5)

gg.create_centroidal_task(des_mom, 0, T, "mom track", 1e4)

xs, us = gg.optimize(x0, stateWeights, x0, wt_xreg=5e-3)
np.savez("../motion_planner/dat_file/ik", xs = xs)
