## This file contains the motion plan for cartwheel
## Author : Avadesh Meduri
## Date : 21/09/2021

import numpy as np
from robot_properties_solo.config import Solo12Config
from weight_abstract import ACyclicMotionParams
import pinocchio as pin
###########################################################################################

robot = Solo12Config.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])


###########################################################################################
plan = ACyclicMotionParams("solo12", "hifive")

st = 0.4
rt = 0.3
jt = 0.3
T = 1.2
dt = 5e-2

plan.cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                [[ 0.,      0.3946,   0.14695,  0.,st, st + rt   ],
                [ 0.,      0.3946,  -0.14695,  0., st, st + rt   ],
                [ 1.,      0.0054,   0.14695,  0., st, st + rt   ],
                [ 1.,      0.0054,  -0.14695,  0., st, st + rt   ]],

                [[ 0.,      0.3946,   0.14695,  0.,st+rt, st+rt+jt   ],
                [ 0.,      0.3946,  -0.14695,  0., st+rt, st+rt+jt   ],
                [ 0.,      0.0054,   0.14695,  0., st+rt, st+rt+jt   ],
                [ 0.,      0.0054,  -0.14695,  0., st+rt, st+rt+jt   ]],

                [[ 1.,      0.3946,   0.14695,  0.,st+rt+jt, T ],
                [ 1.,      0.3946,  -0.14695,  0., st+rt+jt, T ],
                [ 1.,      0.0054,   0.14695,  0., st+rt+jt, T ],
                [ 1.,      0.0054,  -0.14695,  0., st+rt+jt, T ]]]

plan.n_col = 20
plan.dt_arr = plan.n_col*[dt,]

#  dynamic optimization params
plan.W_X =        np.array([1e+3, 1e-5, 1e+5, 1e-4, 1e-4, 1e-4, 3e+3, 7e+2, 3e+1])
plan.W_X_ter = 10*np.array([1e+3, 1e-1, 1e+5, 1e-1, 1e-1, 1e-1, 1e+2, 1e+2, 1e+2])
plan.W_F = np.array(4*[1e+1, 1e+1, 1e+0])
plan.rho = 5e+4

plan.X_nom = [[0.2, 0, 0.22, 0, 0, 0, 0, .05, 0.0, 0.0, st],
              [0.2, 0, 0.25, 0, 0, 0, 0, 0.0, 0., st, st+rt],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st+rt, T]]

plan.X_ter = [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0.0]

plan.bounds = [ [0.25, 0.25, 0.22, 0, st],
                [0.25, 0.25, 0.35, st, st+rt],
                [0.5, 0.5, 0.5, st+rt, st+rt+jt],
                [0.25, 0.25, 0.25, st+rt+jt, T]]

# ik optimization params

plan.cent_wt = [1e3, 1e4]
plan.cnt_wt = 1e4

plan.swing_wt = [[[1e0, 0.3946,   0.14695,  0.6, st + 0.25*rt, st + 0.5*rt],
                  [0, 0.3946,   -0.14695,  0.2, st + 0.25*rt, st + 0.5*rt],
                  [0,  0.0054,   0.14695,  0., st + 0.25*rt, st + 0.5*rt],
                  [0,  0.0054,   -0.14695,  0., st + 0.25*rt, st + 0.5*rt]]]

plan.swing_wt = [[[0, 0.3946,   0.14695,  0.0, st + 0.25*rt, st + 0.5*rt],
                  [0, 0.3946,   -0.14695,  0.0, st + 0.25*rt, st + 0.5*rt],
                  [0,  0.0054,   0.14695,  0., st + 0.25*rt, st + 0.5*rt],
                  [0,  0.0054,   -0.14695,  0., st + 0.25*rt, st + 0.5*rt]]]

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e-2] + [10, 1, 10] + 4*[1e-1, 1e-1, 1e-1] \
                     + [0.00] * 3 + [10, 1, 10] + [0.1] *(rmodel.nv - 6))

plan.state_reg = [np.hstack((x_reg1, [0, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, T]))]
plan.state_scale = [[1e-2, 0, T]]

ctrl_wt = [0, 0, 10] + [1, 1, 1] + [10.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[1e-4, 0, T]]

# controller details 
plan.kp = 7.5
plan.kd = 0.2
