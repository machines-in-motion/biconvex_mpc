## This file is for testing and profiling the pybind functions
## for fista
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

## creating C++ class
prob = fista_py.data(Q_f, q_f, A_x, b_x, P_k, n, 1e+5)

Fista = fista_py.instance()

st = time.time()
obj = prob.compute_obj(F_opt)
# obj_2 = prob.compute_obj_2(F_opt)
grad = prob.compute_grad(F_opt)
et = time.time()
# print(1e3*(et - st))

print(obj)