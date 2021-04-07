## This file is a trial to see how terminal conditions can be used
## Author : Avadesh Meduri
## Date : 7/04/2021

import time
import numpy as np
import pinocchio as pin

from py_biconvex_mpc.motion_planner.biconvex import BiConvexMP
from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics

from cnt_plan_utils import SoloCntGen
from py_biconvex_mpc.bullet_utils.solo_env import Solo12Env

robot = Solo12Config.buildRobotWrapper()
n_eff = 4
m = pin.computeTotalMass(robot.model)
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])

# cnt plan
sT = 0.1
eT = 0.4
dt = 5e-2

cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., sT,  eT    ],
             [ 1.,      0.3946,  -0.14695,  0., sT,  eT    ],
             [ 1.,      0.0054,   0.14695,  0., sT,  eT    ],
             [ 1.,      0.0054,  -0.14695,  0., sT,  eT    ]]]

cnt_plan = np.array(cnt_plan)

# optimal trajactory
f = np.load("dat_file/mom_wm.npz")
X_opt, F_opt, P_opt = f["X_opt"], f["F_opt"], f["P_opt"]


# initial and ter state
X_init = np.reshape(X_opt[9*int(sT/dt):9*(int(sT/dt)+1)],(9))
X_ter = np.reshape(X_opt[9*int(eT/dt):9*(int(eT/dt)+1)],(9))

X_nom = np.zeros((9*int(np.round((eT-sT)/dt,2))))
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

mp = BiConvexMP(m, dt, eT - sT, n_eff, rho = rho)
mp.create_contact_array(cnt_plan)
mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)

mp.create_cost_X(W_X, W_X_ter, X_ter)
mp.create_cost_F(W_F)
st = time.time()
com_opt, F_opt, mom_opt = mp.optimize(X_init, 50)
et = time.time()
print("net time:", et - st)
mp.stats()
