## This contains a pure IK based troting motion plan
## Author : Avadesh Meduri
## Date : 26/02/2021

import time
import numpy as np
import pinocchio as pin

# from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

from cnt_plan_utils import SoloCntGen
from py_biconvex_mpc.bullet_utils.solo_env import Solo12Env


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
st = 0.2 # step time 
sh = 0.15 # step height
sl = np.array([0.1,0.0,0]) # step length
n_steps = 6 # number of steps
T = st*(n_steps + 2)
dt = 5e-2

X_ter[0:3] += sl*(n_steps)
X_nom = np.zeros((9*int(T/dt)))
X_nom[2::9] = X_init[2]

cnt_planner = SoloCntGen(T, dt, gait = 1)
cnt_plan = cnt_planner.create_trot_plan(st, sl, n_steps)

# weights
# W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-2, 1e5, 1e5, 1e5])
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

solve = True

if solve:
    # optimization
    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_contact_array(cnt_plan)
    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
    mp.create_cost_X(W_X, W_X_ter, X_ter, X_nom)
    mp.create_cost_F(W_F)
    st = time.time()
    com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)
    X_opt, P_opt = mp.get_optimal_x_p()

    # print(F_opt[2::3])
    et = time.time()
    print("time", et - st)

    mp.stats()

    np.savez("./dat_file/mom_wm", X_opt = X_opt, F_opt = F_opt, P_opt = P_opt)

else:
    f = np.load("dat_file/mom_wm.npz")
    X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]

    X_wm = X_opt #+ 0*np.random.rand(len(X_opt))[:,None]
    F_wm = F_opt #+ 0*np.random.rand(len(F_opt))[:,None]

    X_init[0:2] = 0.2*np.random.rand(2)
    # print(X_init)

    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
    mp.create_contact_array(cnt_plan)

    mp.create_cost_X(W_X, W_X_ter, X_ter)
    mp.create_cost_F(W_F)
    st = time.time()
    com_opt, F_opt, mom_opt = mp.optimize(X_init, 50, X_wm = X_wm, F_wm = F_wm, P_wm= P_opt)
    et = time.time()
    print("net time:", et - st)
    mp.stats()
    X_opt_n, P_opt_n = mp.get_optimal_x_p()
