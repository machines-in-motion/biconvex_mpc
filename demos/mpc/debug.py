
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

## Motion
gait_params = walk

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

v_des = np.array([0.5,0.0, 0])
step_height = gait_params.step_ht

plan_freq = 0.05 # sec

gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, gait_params, x0, plan_freq, q0)

sim_dt = .001

# q = q0
# v = v0

data = np.load("./dat_file/debug.npz")
q, v = data["q"], data["v"]
step_t = data["t"]
           

for o in range(1):
    contact_configuration = np.array([1,1,1,1])
    # gg.update_params(X_nom=x_wm.copy())
    xs, us, f = gg.optimize(q, v, np.round(step_t,3), v_des, gait_params.step_ht, contact_configuration)

    X_opt = gg.mp.X_opt
    F_opt = gg.mp.F_opt
    gg.plot_plan()                
    # plt.plot(X_opt[2::9], label = "X_opt")
    # plt.plot(x_wm[2::9], label = "wm")
    # plt.legend()
    # plt.show()
    # gg.reset()
    
    q = xs[int(plan_freq/sim_dt)-1][0:pin_robot.model.nq]
    v = xs[int(plan_freq/sim_dt)-1][pin_robot.model.nq:]
    step_t = (step_t + plan_freq)%gait_params.gait_period