## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

from robot_properties_solo.config import Solo12Config

robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])
# initial and ter state
X_init = np.zeros(9)
X_init[0:3] = pin.centerOfMass(robot.model, robot.data, q0,  pin.utils.zero(robot.model.nv))
X_ter = X_init.copy()

# contact plan
dt = 5e-2
v_des = np.array([0.3, 0, 0])
st = 0.25
T = st
eff_arr = ["FL", "FR", "HL", "HR"]
a, b, c, d = 1, 0, 0, 1
cnt_plan = [[[ a*1.,      0.3946,   0.14695,  0., 0,  st ],
             [ b*1.,      0.3946,  -0.14695,  0., 0,  st ],
             [ c*1.,      0.0054,   0.14695,  0., 0,  st ],
             [ d*1.,      0.0054,  -0.14695,  0., 0,  st ]]]

sl_arr = v_des*st

gg = GaitGenerator(robot, Solo12Config.urdf_path, st, dt)
for i in range(4):
    if cnt_plan[0][i][0] == 0:
        gg.create_swing_foot_task(np.array(cnt_plan[0][i][1:4]),np.array(cnt_plan[0][i][1:4]) + sl_arr, 0.0, st - dt, 0.1, eff_arr[i] + "_FOOT", eff_arr[i] + "_step", 1e5)
        gg.create_contact_task(np.array(cnt_plan[0][i][1:4])+ sl_arr, st - dt, st, eff_arr[i] + "_FOOT", eff_arr[i] + "_step", 1e6)

    else:
        gg.create_contact_task(np.array(cnt_plan[0][i][1:4]), 0.0, st, eff_arr[i] + "_FOOT", eff_arr[i] + "_step", 1e6)

X_ter[0:3] = X_init[0:3] + v_des*st
X_ter[3:6] = v_des
X_nom = np.zeros((9*int(T/dt)))

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 3e3, 3e3, 3e3])

W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5])

W_F = np.array(4*[1e+1, 1e+1, 1e+1])

rho = 5e+4 # penalty on dynamic constraint violation

# constraints 
bx = 0.25
by = 0.25
bz = 0.25
fx_max = 15
fy_max = 15
fz_max = 15

mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
mp.create_contact_array(np.array(cnt_plan))
mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
mp.create_cost_X(W_X, W_X_ter, X_ter, X_nom)
mp.create_cost_F(W_F)
t1 = time.time()
com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)

gg.create_centroidal_task(mom_opt, 0, st, "mom_track", 5e2)
gg.create_centroidal_task(mom_opt[-1], st, st, "mom_track", 1e3, True)
gg.ik.add_com_position_tracking_task(0, st, com_opt, 1e2, "com_track_cost", False)
gg.ik.add_com_position_tracking_task(0, st, com_opt[-1], 1e2, "com_track_cost", True)

state_wt = np.array([0.] * 3 + [1000.] * 3 + [5.0] * (robot.model.nv - 6) \
                        + [0.01] * 6 + [5.0] *(robot.model.nv - 6))
xs, us = gg.optimize(x0, state_wt, x0, wt_xreg=7e-3)
t2 = time.time()
print("time", t2 - t1)
com_opt_ik, mom_opt_ik = gg.compute_optimal_com_and_mom()
mp.add_ik_com_cost(com_opt_ik)
mp.add_ik_momentum_cost(mom_opt_ik) 
# mp.stats()
np.savez("../motion_planner/dat_file/ik", xs = xs)
