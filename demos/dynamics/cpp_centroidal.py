## This file is a unit test for c++ implementation of centroidal dynamics
## Author : Avadesh Meduri 
## Date : 16/05/2021

import numpy as np
from biconvex_mpc_cpp import BiconvexMP, CentroidalDynamics
from test_problem import test_problem_func

mp, cnt_plan, X_init, X_ter = test_problem_func()
mp_cpp = BiconvexMP(mp.m, mp.dt, mp.T, mp.n_eff)
for i in range(cnt_plan.shape[0]):
    mp_cpp.set_contact_plan(cnt_plan[i])

f = np.load("../motion_planner/dat_file/mom_wm.npz")
X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]
A_x, b_x = mp.compute_X_mat(X_opt, mp.r_arr, mp.cnt_arr)
A_f, b_f = mp.compute_F_mat(F_opt, mp.r_arr, mp.cnt_arr, X_init)

mp_cpp.print_contact_array()
A_x_cpp = mp_cpp.return_A_x(X_opt)
A_f_cpp = mp_cpp.return_A_f(F_opt, X_init)
b_x_cpp = mp_cpp.return_b_x(X_opt)
b_f_cpp = mp_cpp.return_b_f(F_opt, X_init)


# unit tests for dynamic matrix computation
assert np.linalg.norm(A_x - A_x_cpp) == 0
assert np.linalg.norm(A_f - A_f_cpp) == 0
assert np.linalg.norm(b_x - b_x_cpp) == 0
assert np.linalg.norm(b_f - b_f_cpp) == 0

