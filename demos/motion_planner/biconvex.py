# Demo of the Biconvex solver
# Author : Avadesh Meduri
# Date : 7/12/2020

import os.path
import sys
import numpy as np
from matplotlib import pyplot as plt

curdir = os.path.dirname(__file__)
cdir = os.path.abspath(os.path.join(curdir,'../../python/'))
sys.path.append(cdir)

from motion_planner.biconvex import BiConvexMP

# params
m = 2
dt = 0.1
T = 0.3
n_eff = 2
n_col = int(np.round(T/dt, 2))

# weights
W_X = np.array(9*[1e-5])
W_X_ter = np.array([1e+1, 1e+1, 1e+1, 1e+1, 1e+1, 1e+1, 1e+2, 1e+2, 1e+2])

W_F = np.array([1e+2, 1e+2, 1e+1, 1e+2, 1e+2, 1e+1])

# initial and ter state
X_init = np.array([0, 0, 0.2, 0, 0, 0, 0, 0, 0])
X_ter = np.array([0, 0, 0.4, 0, 0, 0, 0, 0, 0])

# contact plan
cnt_plan = np.array([[[1, 0, 0.5, 0, 0, 0.1], [0, 0, 0.0, 0, 0, 0.1]],
                     [[0, 0, 0.0, 0, 0.1, 0.2], [1, 0, -0.5, 0, 0.1, 0.2]],
                     [[1, 0, 0.5, 0, 0.2, 0.3], [1, 0, -0.5, 0, 0.2, 0.3]]])

mp = BiConvexMP(m, dt, T, n_eff)
mp.create_contact_array(cnt_plan)
# print(mp.r_arr)
# mp.create_cost_X(W_X, W_X_ter, X_ter)
# mp.create_cost_F(W_F)
# print(mp.Q_F)
