# Demo of the Biconvex solver
# Author : Avadesh Meduri
# Date : 7/12/2020

import os.path
import sys
import numpy as np
from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from cnt_plan_utils import SoloCntGen

# params
m = 2.4
dt = 0.05
n_eff = 4

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e+3, 1e+3, 1e+3])
W_X_ter = np.array([1e+5, 1e+5, 1e+5, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4])

W_F = np.array(2*[1e+1, 1e+1, 1e+1, 1e+1, 1e+1, 1e+1])

rho = 1e+3 # penalty on dynamic constraint violation

# initial and ter state
X_init = np.array([0, 0, 0.2, 0, 0, 0, 0, 0, 0])
X_ter = np.array([1.0, 0, 0.2, 0, 0, 0, 0, 0, 0])

# contact plan
st = 0.2 # step time 
sl = np.array([0.2,0.0,0]) # step length
n_steps = 5 # number of steps
T = st*(n_steps + 2)

cnt_planner = SoloCntGen(0.2, 0.2)
cnt_plan = cnt_planner.create_trot_plan(st, sl, n_steps)
# print(cnt_plan)

# assert False
# constraints 
bx = 0.2
by = 0.2
bz = 0.32
fx_max = 15
fy_max = 15
fz_max = 15

# optimization
mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
mp.create_contact_array(cnt_plan)
mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)

X_opt, F_opt = mp.optimize(X_init, X_ter, W_X, W_F, W_X_ter, 20)
# print(F_opt[2::3])
mp.stats()
