## This file is for testing and profiling the pybind functions
## for fista
## Author : Avadesh Meduri & Paarth Shah
## Date : 7/04/2021

from scipy.sparse import csc_matrix
import time
import numpy as np
from test_problem import test_problem_func
import fista_py

import osqp


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

L0 = 150.0
beta = 1.5
max_iters = int(150)
rho = 1e5
tol = 1e-5

#Initial guess for F
F_test = np.random.rand(F_opt.shape[0])[:,None]
X_test = np.random.rand(X_opt.shape[0])[:,None]

#Set up and solve using FISTA C++
solver = fista_py.instance(L0, beta, max_iters, tol)
solver.set_data(Q_f, q_f, A_x, b_x, P_k, F_low, F_high, rho, n)
F_c = solver.optimize(F_test)[:,None]

solver_x = fista_py.instance(L0, beta, max_iters, tol)
solver_x.set_data(Q_x, q_x, A_f, b_f, P_k, X_low, X_high, rho, n)
X_c = solver_x.optimize(X_test)[:,None]
# assert False
# Setting up FISTA python
obj_f = lambda f : f.T *Q_f*f + q_f.T*f + rho *np.linalg.norm(A_x*f - b_x + P_k)**2    
grad_obj_f = lambda f: 2*Q_f*f + q_f + 2.0*rho*A_x.T*(A_x*f - b_x + P_k)
grad_py = grad_obj_f(F_test)
obj_py = obj_f(F_test)
proj_f = lambda f, L : np.clip(f, F_low, F_high)

# settin up FISTA for X
obj_x = lambda x : x.T *Q_x*x + q_x.T*x + rho*np.linalg.norm(A_f*x - b_f + P_k)**2    
grad_obj_x = lambda x: 2.0*Q_x*x + q_x + 2.0*rho*A_f.T*(A_f*x - b_f + P_k)
proj_x = lambda x, L : np.clip(x, X_low, X_high)

# # mp.fista.L0 = L0
F_k_1 = mp.fista.optimize(obj_f, grad_obj_f, proj_f, F_test, max_iters, tol)
# print(F_k_1.shape, F_c.shape, A_x.shape, b_x.shape)
print("norm F", np.linalg.norm(F_c - F_k_1))

X_k_1 = mp.fista.optimize(obj_x, grad_obj_x, proj_x, X_test, max_iters, tol)
print("norm x", np.linalg.norm(X_c - X_k_1))


# mp.fista.L0 = L0
F_k_2 = mp.fista.optimize(obj_f, grad_obj_f, proj_f, F_test, max_iters, tol)
# print(F_k_1.shape, F_c.shape, A_x.shape, b_x.shape)
print("norm F", np.linalg.norm(F_c - F_k_2))
print("norm F", np.linalg.norm(F_k_2 - F_k_1))

#Set up and solve using OSQP
# A_x_ineq = np.identity(A_x.shape[1])
# A_sparse = csc_matrix(A_x_ineq)
# Q_sparse = csc_matrix(np.matrix(2*Q_f + 2*rho*(A_x.T)*A_x))
# z_x = -b_x + P_k

# lb = F_low[0:A_sparse.shape[0]]
# ub = F_high[0:A_sparse.shape[0]]

# osqp_solver = osqp.OSQP()
# osqp_solver.setup(Q_sparse, q_f + 2*rho*A_x.T*z_x, A_sparse, lb, ub, verbose=False,
#              eps_abs=1e-5, eps_rel=1e-5, eps_prim_inf=1e-5, eps_dual_inf=1e-5,
#              check_termination = 5, max_iter = 200, polish=False, scaling = 1)
# result = osqp_solver.solve()
# # print(result.x)
# osqp_solve_time = result.info.solve_time
# print("OSQP solve time: " + str(osqp_solve_time*1000)+"ms")
