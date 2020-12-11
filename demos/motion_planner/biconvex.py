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
T = 1.2
n_eff = 2
n_col = int(np.round(T/dt, 2))

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e+1, 1e+1, 1e+1])
W_X_ter = np.array([1e+5, 1e+5, 1e+5, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4])

W_F = np.array([1e+1, 1e+1, 1e+1, 1e+1, 1e+1, 1e+1])

rho = 1e+3 # penalty on dynamic constraint violation

# initial and ter state
X_init = np.array([0, 0, 0.2, 0, 0, 0, 0, 0, 0])
X_ter = np.array([1.0, 0, 0.2, 0, 0, 0, 0, 0, 0])

# contact plan
step_time = 0.2
sl = 0.2 # step length
n_ee = 4
# cnt_plan = np.array([[[1,0.0,-0.2,0,0*step_time,(0+1)*step_time], [0,0.0,-0.2,0,0*step_time,(0+1)*step_time]],
#                      [[0,sl,-0.2,0,1*step_time,(1+1)*step_time], [1,1*sl,-0.2,0,1*step_time,(1+1)*step_time]],
#                      [[1,2*sl,-0.2,0,2*step_time,(2+1)*step_time], [0,2*sl,-0.2,0,2*step_time,(2+1)*step_time]],
#                      [[0,3*sl,-0.2,0,3*step_time,(3+1)*step_time], [1,3*sl,-0.2,0,3*step_time,(3+1)*step_time]],
#                      [[1,4*sl,-0.2,0,4*step_time,(4+1)*step_time], [0,4*sl,-0.2,0,4*step_time,(4+1)*step_time]],
#                      [[0,5*sl,-0.2,0,5*step_time,(5+1)*step_time], [1,5*sl,-0.2,0,5*step_time,(5+1)*step_time]]])

cnt_plan = np.array([[[1,0.0,-0.2,0,0*step_time,T], [1,0.0,0.2 ,0,0*step_time,T]]])



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
# print(mp.cnt_arr)
# mp.create_cost_X(W_X, W_X_ter, X_ter)
# mp.create_cost_F(W_F)
# print(mp.Q_F)

X_opt, F_opt = mp.optimize(X_init, X_ter, W_X, W_F, W_X_ter, 12)
# print(F_opt[2::3])
mp.stats()
