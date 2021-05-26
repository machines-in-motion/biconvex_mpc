import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

from mpc_gait_gen import SoloMpcGaitGen

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
st = 0.25
dt = 5e-2
state_wt = np.array([0.] * 3 + [1e5] * 3 + [5.0] * (pin_robot.model.nv - 6) \
                        + [0.01] * 3 + [1.5] * 3 + [5.0] *(pin_robot.model.nv - 6))

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

gg = SoloMpcGaitGen(pin_robot, urdf_path, st, dt, state_wt, x0, plan_freq, gait = 1)

next_loc = np.array([[ 0.3946 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.3946 + sl_arr[0],  -0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                    [ 0.0054 + sl_arr[0],  -0.14695 + sl_arr[1], 0]])

tmop = gg.create_cnt_plan(q0, v0, t, 1, next_loc, v_des)

print(tmop)