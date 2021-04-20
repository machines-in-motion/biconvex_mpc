## this file is to test if the pybind works properly
## Author : Avadesh Meduri
## Date : 19/04/2021

import numpy as np
from scipy.sparse import csc_matrix
from biconvex_mpc_cpp import BiconvexMP, CentroidalDynamics
from test_problem import test_problem_func

mp, cnt_plan, X_init, X_ter = test_problem_func()
mp_cpp = BiconvexMP(mp.m, mp.dt, mp.T, mp.n_eff)
for i in range(cnt_plan.shape[0]):
    mp_cpp.set_contact_plan(cnt_plan[i])

f = np.load("../motion_planner/dat_file/mom_wm.npz")
X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]

X_k = X_opt
F_k = np.random.rand(len(F_opt))[:,None]
P_k = P_opt
max_iters = 150
tol = 1e-5


A_x, b_x = mp.compute_X_mat(X_k, mp.r_arr, mp.cnt_arr)


print("make sure this is called in c++ also")
mp_cpp.create_contact_array()

mp_cpp.set_cost_x(csc_matrix(mp.Q_X), mp.q_X)
mp_cpp.set_cost_f(csc_matrix(mp.Q_F), mp.q_F)
mp_cpp.set_bounds_x(mp.X_low, mp.X_high)
mp_cpp.set_bounds_f(mp.F_low, mp.F_high)


mp_cpp.set_warm_start_vars(X_k, F_k, P_k)

mp_cpp.optimize(X_init, 50)
# python checks
# obj_f = lambda f : f.T *mp.Q_F*f + mp.q_F.T*f + mp.rho *np.linalg.norm(A_x*f - b_x + P_k)**2    
# grad_obj_f = lambda f: 2*mp.Q_F*f + mp.q_F + 2.0*mp.rho*A_x.T*(A_x*f - b_x + P_k)
# proj_f = lambda f, L : np.clip(f, mp.F_low, mp.F_high)


# print(obj_f(F_k))

# F_k_1 = mp.fista.optimize(obj_f, grad_obj_f, proj_f, F_k, max_iters, tol)
# print(mp.fista.k)
# print("norm", np.linalg.norm(tmp[:,None] - F_k_1))
# print("obj", obj_x(X_k))

# optimizing x
# A_f, b_f = mp.compute_F_mat(F_k_1, mp.r_arr, mp.cnt_arr, X_init)

# obj_x = lambda f : f.T *mp.Q_X*f + mp.q_X.T*f + mp.rho *np.linalg.norm(A_f*f - b_f + P_k)**2    
# grad_obj_x = lambda f: 2*mp.Q_X*f + mp.q_X + 2.0*mp.rho*A_f.T*(A_f*f - b_f + P_k)
# proj_x = lambda f, L : np.clip(f, mp.X_low, mp.X_high)

# X_k_1 = mp.fista.optimize(obj_x, grad_obj_x, proj_x, X_k, max_iters, tol)
# print(mp.fista.k)
# print("norm", np.linalg.norm(tmp[:,None] - X_k_1))

