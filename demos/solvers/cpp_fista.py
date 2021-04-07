## This file is for testing and profiling the pybind functions
## for fista
## Author : Avadesh Meduri & Paarth Shah
## Date : 7/04/2021

import numpy as np
from test_problem import test_problem_func
import fista_py


f = np.load("../motion_planner/dat_file/mom_wm.npz")
X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]

mp = test_problem_func()

A_x, b_x = mp.compute_X_mat(X_opt, mp.r_arr, mp.cnt_arr)
A_f, b_f = mp.compute_F_mat(F_opt, mp.r_arr, mp.cnt_arr, np.reshape(X_opt[0:9],9))

## creating C++ class

prob = fista_py.problem(A_x)