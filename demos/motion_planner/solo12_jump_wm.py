## This file contains a jump demo with warm start
## Author : Avadesh Meduri
## Date : 12/03/2021

import time
import numpy as np
import pinocchio as pin

# from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics

from cnt_plan_utils import SoloCntGen
#from py_biconvex_mpc.bullet_utils.solo_env import Solo12Env

robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])

# cnt plan
rt = 0.4 # reartime 
T = 0.4 + rt + 0.1
dt = 5e-2

cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  0.4    ],
             [ 1.,      0.3946,  -0.14695,  0., 0.,  0.4    ],
             [ 1.,      0.0054,   0.14695,  0., 0.,  0.4    ],
             [ 1.,      0.0054,  -0.14695,  0., 0.,  0.4    ]],
        
            [[ 0.,      0.3946,   0.14695,  0., 0.4, 0.4 + rt   ],
             [ 0.,      0.3946,  -0.14695,  0., 0.4, 0.4 + rt   ],
             [ 0.,      0.0054,   0.14695,  0., 0.4, 0.4 + rt   ],
             [ 0.,      0.0054,  -0.14695,  0., 0.4, 0.4 + rt   ]],
        
            [[ 1.,      0.3946,   0.14695,  0., 0.4 + rt, T    ],
             [ 1.,      0.3946,  -0.14695,  0., 0.4 + rt, T    ],
             [ 1.,      0.0054,   0.14695,  0., 0.4 + rt, T    ],
             [ 1.,      0.0054,  -0.14695,  0., 0.4 + rt, T    ]]]

cnt_plan = np.array(cnt_plan)

# initial and ter state
X_init = np.zeros(9)
X_init[0:3] = pin.centerOfMass(robot.model, robot.data, q0,  pin.utils.zero(robot.model.nv))
X_ter = X_init.copy()

X_nom = np.zeros((9*int(np.round(T/dt,2))))
X_nom[2::9] = X_init[2]

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 5e2, 5e2, 5e2])

W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5])

W_F = np.array(4*[1e+1, 1e+1, 1e+1])

rho = 1e+5 # penalty on dynamic constraint violation

# constraints 
bx = 0.25
by = 0.25
bz = 0.25
fx_max = 20
fy_max = 20
fz_max = 20

# optimization
solve = True
if solve:
    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_contact_array(cnt_plan)
    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)

    mp.create_cost_X(W_X, W_X_ter, X_ter)
    mp.create_cost_F(W_F)
    st = time.time()
    com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)
    et = time.time()
    print("net time:", et - st)
    X_opt, P_opt = mp.get_optimal_x_p()
    mp.stats()

    np.savez("./dat_file/mom_wm", X_opt = X_opt, F_opt = F_opt, P_opt = P_opt)
    np.savez("./dat_file/mom", com_opt = com_opt, mom_opt = mom_opt, F_opt = F_opt)

else:
    f = np.load("dat_file/mom_wm.npz")
    X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]

    X_wm = X_opt #+ 0*np.random.rand(len(X_opt))[:,None]
    F_wm = F_opt #+ 0*np.random.rand(len(F_opt))[:,None]

    X_init[0:2] = 0.2*np.random.rand(2)
    print(X_init)

    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)

    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
    mp.create_contact_array(cnt_plan)

    mp.create_cost_X(W_X, W_X_ter, X_ter)
    mp.create_cost_F(W_F)
    st = time.time()
    com_opt, F_opt, mom_opt = mp.optimize(X_init, 50, X_wm = X_wm, F_wm = F_wm, P_wm= P_opt)
    et = time.time()
    print("net time:", et - st)
    mp.stats()
    X_opt_n, P_opt_n = mp.get_optimal_x_p()
