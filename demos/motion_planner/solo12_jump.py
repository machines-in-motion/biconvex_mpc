## This file contains a rearing demo for solo12
## Author : Avadesh Meduri
## Date : 12/03/2021

import time
import numpy as np
import pinocchio as pin

from inverse_kinematics_cpp import InverseKinematics
from biconvex_mpc_cpp import BiconvexMP, KinoDynMP

from robot_properties_solo.config import Solo12Config

from cnt_plan_utils import SoloCntGen
from py_biconvex_mpc.bullet_utils.solo_env import Solo12Env

robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])

# cnt plan
rt = 0.5 # reartime 
T = 0.5 + rt + 0.1
dt = 5e-2

cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  0.5    ],
             [ 1.,      0.3946,  -0.14695,  0., 0.,  0.5    ],
             [ 1.,      0.0054,   0.14695,  0., 0.,  0.5    ],
             [ 1.,      0.0054,  -0.14695,  0., 0.,  0.5    ]],
        
            [[ 1.,      0.3946,   0.14695,  0., 0.5, 0.5 + rt   ],
             [ 1.,      0.3946,  -0.14695,  0., 0.5, 0.5 + rt   ],
             [ 1.,      0.0054,   0.14695,  0., 0.5, 0.5 + rt   ],
             [ 1.,      0.0054,  -0.14695,  0., 0.5, 0.5 + rt   ]],
        
            [[ 1.,      0.3946,   0.14695,  0., 0.5 + rt, T    ],
             [ 1.,      0.3946,  -0.14695,  0., 0.5 + rt, T    ],
             [ 1.,      0.0054,   0.14695,  0., 0.5 + rt, T    ],
             [ 1.,      0.0054,  -0.14695,  0., 0.5 + rt, T    ]]]

cnt_plan = np.array(cnt_plan)

# initial and ter state
X_init = np.zeros(9)
X_init[0:3] = pin.centerOfMass(robot.model, robot.data, q0,  pin.utils.zero(robot.model.nv))
X_init[4] = 0.5
X_ter = X_init.copy()

X_nom = np.zeros((9*int(np.round(T/dt,2))))
X_nom[2::9] = X_init[2]

# weights
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 3e3, 3e3, 3e3])

W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5])

W_F = np.array(4*[1e+1, 1e+1, 1e+1])

rho = 1e+5 # penalty on dynamic constraint violation

# constraints 
bx = 0.25
by = 0.25
bz = 0.25
fx_max = 21
fy_max = 21
fz_max = 21

# optimization
optimize = False

if optimize :
    mom_opt_ik = None
    com_opt_ik = None
    for k in range(1):
        mp = BiconvexMP(m, dt, int(T/dt), n_eff)
        mp.set_rho(rho)
        mp.create_contact_array(cnt_plan)
        mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)

        # mp.add_via_point([0.00, 0.0, 0.05], 0.1, [1e-5, 1e-5, 1e+5])

        mp.create_cost_X(W_X, W_X_ter, X_ter)
        mp.create_cost_F(W_F)
        t1 = time.time()
        com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)
        t2 = time.time()

        cnt_planner = SoloCntGen(T, dt, gait = 1)
        cnt_planner.create_contact_costs(cnt_plan, 5e5)
        # cnt_planner.create_com_tasks(mom_opt, com_opt, [5e+6, 1e+3], [5e+6, 1e+3])
        cnt_planner.create_com_tasks(mom_opt, com_opt, [1e+2, 1e+1], [1e+2, 1e+1])
        ik_solver = cnt_planner.return_gait_generator()

        state_wt = np.array([0.] * 3 + [100.] * 3 + [8.0] * (robot.model.nv - 6) \
                            + [0.01] * 6 + [4.0] *(robot.model.nv - 6))
        t3 = time.time()

        xs, us = ik_solver.optimize(x0, x_reg  = x0, wt_xreg=7e-3, wt_ureg = 3e-3, state_wt=state_wt)
        t4 = time.time()
        print("mp time", t2 - t1)
        print("ik time", t2 - t1)

        com_opt_ik, mom_opt_ik = ik_solver.compute_optimal_com_and_mom()

        W_X = np.array([1e+3, 1e+3, 1e+3, 1e+3, 1e+3, 1e+3, 3e3, 3e3, 3e3])
        
        # adding center of mass and momentum tracking cost
        mp.add_ik_com_cost(com_opt_ik)
        mp.add_ik_momentum_cost(mom_opt_ik)
        mp.stats()

    np.savez("./dat_file/mom", com_opt = com_opt, mom_opt = mom_opt, F_opt = F_opt)
    np.savez("./dat_file/ik", xs = xs, us = us)
    np.savetxt("./dat_file/jump.txt", xs[:,0:19])


else:
    f = np.load("dat_file/mom.npz")
    mom_opt, com_opt, F_opt = f["mom_opt"], f["com_opt"], f["F_opt"]
    f = np.load("dat_file/ik.npz")
    xs = f["xs"]
    us = f['us']

    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_contact_array(cnt_plan)
    
    # simulation
    kp = 4*[2.5, 50.0, 50.0]
    kd = 4*[.02, .0, .0]
    kc = [400, 400, 400]
    dc = [50,50,50]
    kb = [100, 100, 100]
    db = [10,10,10]
    env = Solo12Env(X_init, T, dt, kp, kd, kc, dc, kb, db)
    env.generate_motion_plan(com_opt, mom_opt, F_opt, mp.cnt_arr.copy(), mp.r_arr.copy())
    env.generate_end_eff_plan(xs, us)
    # env.plot()
    env.sim(fr = 0.00, vname = None)
    # env.plot_real()