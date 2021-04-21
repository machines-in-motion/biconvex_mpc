## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP

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
st = 0.2
a, b, c, d = 1, 0, 0, 1
cnt_plan = [[[ a*1.,      0.3946,   0.14695,  0., 0,  st ],
             [ b*1.,      0.3946,  -0.14695,  0., 0,  st ],
             [ c*1.,      0.0054,   0.14695,  0., 0,  st ],
             [ d*1.,      0.0054,  -0.14695,  0., 0,  st ]]]

v_des = 0.2
T = st

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
st = time.time()
com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)
et = time.time()
print("time", et - st)
mp.stats()
