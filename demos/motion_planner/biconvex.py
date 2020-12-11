# Demo of the Biconvex solver
# Author : Avadesh Meduri
# Date : 7/12/2020

import os.path
import sys
import numpy as np
from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP

# params
m = 2
dt = 0.1
T = 1.0
n_eff = 2
n_col = int(np.round(T/dt, 2))

# weights
W_X = np.array(9*[1e-5])
W_X_ter = np.array([1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4])

W_F = np.array([1e+1, 1e+1, 1e+1, 1e+1, 1e+1, 1e+1])

rho = 1e+3 # penalty on dynamic constraint violation

# initial and ter state
X_init = np.array([0, 0, 0.2, 0, 0, 0, 0, 0, 0])
X_ter = np.array([0, 0, 0.5, 0, 0, 0, 0, 0, 0])

# contact plan
cnt_plan = np.array([[[1, 0, 0.5, 0, 0, 0.5], [0, 0, -0.5, 0, 0, 0.5]],
                     [[0, 0, 0.5, 0, 0.5, 1.0], [1, 0, -0.5, 0, 0.5, 1.0]]])

mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
mp.create_contact_array(cnt_plan)
# print(mp.cnt_arr)
# mp.create_cost_X(W_X, W_X_ter, X_ter)
# mp.create_cost_F(W_F)
# print(mp.Q_F)
X_opt, F_opt = mp.optimize(X_init, X_ter, W_X, W_F, W_X_ter, 10)
# print(F_opt[2::3])
mp.stats()
