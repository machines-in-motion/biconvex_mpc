## This contains a pure IK based troting motion plan
## Author : Avadesh Meduri
## Date : 26/02/2021

import time
import numpy as np
import pinocchio as pin

from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

from cnt_plan_utils import SoloCntGen
from py_biconvex_mpc.bullet_utils.solo_env import Solo12Env


robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])
# initial and ter state
X_init = np.zeros(9)
X_init[0:3] = pin.centerOfMass(robot.model, robot.data, q0,  pin.utils.zero(robot.model.nv))
X_ter = X_init.copy()
# contact plan
st = 0.2 # step time 
sh = 0.15 # step height
sl = np.array([0.1,0.0,0]) # step length
n_steps = 6 # number of steps
T = st*(n_steps + 2)
dt = 5e-2

X_ter[0:3] += sl*(n_steps)
X_nom = np.zeros((9*int(T/dt)))
X_nom[2::9] = X_init[2]

cnt_planner = SoloCntGen(T, dt, gait = 1)
cnt_plan = cnt_planner.create_trot_plan(st, sl, n_steps)

# weights
# W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-2, 1e5, 1e5, 1e5])
W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 3e3, 3e3, 3e3])

W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5])

W_F = np.array(4*[1e+1, 1e+1, 1e+1])

rho = 1e+5 # penalty on dynamic constraint violation

# constraints 
bx = 0.25
by = 0.25
bz = 0.25
fx_max = 15
fy_max = 15
fz_max = 15

optimize = False

if optimize:
    # optimization

    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_contact_array(cnt_plan)
    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
    mp.create_cost_X(W_X, W_X_ter, X_ter, X_nom)
    mp.create_cost_F(W_F)
    com_opt, F_opt, mom_opt = mp.optimize(X_init, 30)
    # print(F_opt[2::3])

    cnt_planner.create_ik_step_costs(cnt_plan, sh, [1e+5, 1e+6])
    cnt_planner.create_com_tasks(mom_opt, com_opt, [1e+4, 1e+3], [1e+4, 1e+3])
    ik_solver = cnt_planner.return_gait_generator()
    state_wt = np.array([0.] * 3 + [500.] * 3 + [5.0] * (robot.model.nv - 6) \
                        + [0.01] * 6 + [5.0] *(robot.model.nv - 6))

    xs, us = ik_solver.optimize(x0, wt_xreg=7e-3, state_wt=state_wt)
    com_opt_ik, mom_opt_ik = ik_solver.ik.compute_optimal_com_and_mom()

    W_X = np.array([1e-3, 1e-3, 1e-3, 1e+3, 1e+3, 1e+3, 3e3, 3e3, 3e3])

    mp.add_ik_com_cost(com_opt_ik)
    mp.add_ik_momentum_cost(mom_opt_ik)    
    mp.stats()

    np.savez("./dat_file/mom", com_opt = com_opt, mom_opt = mom_opt, F_opt = F_opt)
    np.savez("./dat_file/ik", xs = xs, us = us)
else:
    # simulation
    f = np.load("dat_file/mom.npz")
    mom_opt, com_opt, F_opt = f["mom_opt"], f["com_opt"], f["F_opt"]
    f = np.load("dat_file/ik.npz")
    xs = f["xs"]
    us = f['us']

    mp = BiConvexMP(m, dt, T, n_eff, rho = rho)
    mp.create_contact_array(cnt_plan)

    kp = 4*[100.0, 100.0, 100.0]
    kd = 4*[.5, 0.5, 0.5]
    kc = [100, 100, 100]
    dc = [10,10,10]
    kb = [200, 200, 200]
    db = [50,50,50]
    env = Solo12Env(X_init, T, dt, kp, kd, kc, dc, kb, db)
    env.generate_motion_plan(com_opt, mom_opt, F_opt, mp.cnt_arr.copy(), mp.r_arr.copy())
    env.generate_end_eff_plan(xs, us)
    # env.plot()
    env.sim(fr = 0.00, vname = None)
    env.plot_real()