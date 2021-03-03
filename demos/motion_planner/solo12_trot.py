## This contains a pure IK based troting motion plan
## Author : Avadesh Meduri
## Date : 26/02/2021

import time
import numpy as np
import pinocchio as pin

from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

from cnt_plan_utils import SoloCntGen

robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)

# initial and ter state
X_init = np.zeros(9)
X_init[0:3] = q0[0:3]
X_ter = X_init.copy()

# contact plan
st = 0.2 # step time 
sh = 0.1 # step height
sl = np.array([0.05,0.0,0]) # step length
n_steps = 4 # number of steps
T = st*(n_steps + 2)
dt = 5e-2

X_ter[0:3] += sl*(n_steps)

cnt_planner = SoloCntGen(T, dt, gait = 1)
cnt_plan = cnt_planner.create_trot_plan(st, sl, n_steps)
ik_solver = cnt_planner.create_ik_step_costs(cnt_plan, sh, [1e2, 1e3])

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e+1, 2e+4, 2e+4, 1e+3, 1e+3, 1e+3])
W_X_ter = np.array([1e+5, 1e+5, 1e+5, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4])

W_F = np.array(4*[1e-1, 1e-1, 1e-1])

rho = 5e+2 # penalty on dynamic constraint violation

# constraints 
bx = 0.25
by = 0.25
bz = 0.25
fx_max = 15
fy_max = 15
fz_max = 15

# optimization
mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
mp.create_contact_array(cnt_plan)
mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)

X_opt, F_opt, mom_opt = mp.optimize(X_init, X_ter, W_X, W_F, W_X_ter, 30)
# print(F_opt[2::3])
mp.stats()

# ik_solver.create_centroidal_task(mom_opt, 0, T, "mom_track_cost", 1e3)
# xs = ik_solver.optimize(x0)