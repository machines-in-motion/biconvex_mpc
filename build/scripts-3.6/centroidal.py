# Demo of the centroidal dynamics class (checks if the matrices are computes properly)
# Author : Avadesh Meduri
# Date : 6/12/2020

import os.path
import sys
import numpy as np
from matplotlib import pyplot as plt

from py_biconvex_mpc.dynamics.centroidal import CentroidalDynamics


m = 2
dt = 0.1
T = 0.3
n_eff = 2
n_col = int(np.round(T/dt, 2))

## X, F

X = np.ones(9*n_col + 9)
F = np.ones(n_eff*3*n_col)
# X[0::2] = 2.0
## r, cnt_plan
r = np.array([[[0.5, 0, 0], [-0.0, 0, 0]],
              [[0.0, 0, 0], [-0.5, 0, 0]],
              [[0.0, 0, 0], [-0.0, 0, 0]]])

cnt_plan = np.array([[1,0],
                     [0,1],
                     [1,1]])

dyn = CentroidalDynamics(m, dt, T, n_eff)
A_F_mat, b_F_mat = dyn.compute_F_mat(F, r, cnt_plan)
A_X_mat, b_X_mat = dyn.compute_X_mat(X, r, cnt_plan)

# print(A_F_mat[18:27,27:36])
# print(b_F_mat)

# print(A_X_mat[9:18, 6:12])
# print(b_X_mat)