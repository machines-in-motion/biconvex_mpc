## This file is a unit test for c++ implementation of centroidal dynamics
## Author : Avadesh Meduri 
## Date : 16/05/2021

from biconvex_mpc_cpp import BiconvexMP
from test_problem import test_problem_func

mp = test_problem_func()

cnt_plan = mp.cnt_arr
r_arr = mp.r_arr

print(cnt_plan.shape, r_arr.shape)

mp_cpp = BiconvexMP(mp.m, mp.dt, mp.T, mp.n_eff)
mp_cpp.set_contact_plan(r_arr, cnt_plan)