## this is for profiling prob data
## Author : Avadesh Meduri & Paarth Shah
## Date : 7/04/2021

from scipy.sparse import csc_matrix
import time
import numpy as np
from test_problem import test_problem_func
import fista_py

f = np.load("../motion_planner/dat_file/mom_wm.npz")
X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]

mp = test_problem_func()

A_x, b_x = mp.compute_X_mat(X_opt, mp.r_arr, mp.cnt_arr)
A_f, b_f = mp.compute_F_mat(F_opt, mp.r_arr, mp.cnt_arr, np.reshape(X_opt[0:9],9))
Q_x = mp.Q_X
q_x = mp.q_X
Q_f = mp.Q_F
q_f = mp.q_F
n = len(F_opt)
P_k = np.ones(A_x.shape[0])[:,None]

F_low = mp.F_low
F_high = mp.F_high
X_low = mp.X_low
X_high = mp.X_high

L0 = 506.25#150.0
beta = 1.5
max_iters = int(150)
rho = 1e5

#Initial guess for F
# F_test = np.random.rand(F_opt.shape[0])[:,None]
F_test = np.ones(F_opt.shape[0])[:,None]
prob = fista_py.data(Q_f, q_f, A_x, b_x, P_k, n, rho)

st = time.time()
obj = prob.compute_obj(F_test)
grad = prob.compute_grad(F_test)
et = time.time()
# print(1e3*(et - st))
print(obj)
