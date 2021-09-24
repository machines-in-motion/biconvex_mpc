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
plan = ACyclicMotionParams("solo12", "cartwheel")

st = 0.3
rt = 0.3
T = 0.8
dt = 5e-2

plan.cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                [[ 0.,      0.3946,   0.14695,  0.,st, st + rt   ],
                [ 0.,      0.3946,  -0.14695,  0., st, st + rt   ],
                [ 0.,      0.0054,   0.14695,  0., st, st + rt   ],
                [ 0.,      0.0054,  -0.14695,  0., st, st + rt   ]],

                [[ 1.,      0.3946,   0.14695,  0.,st + rt, T ],
                [ 1.,      0.3946,  -0.14695,  0., st + rt, T ],
                [ 1.,      0.8054,   0.14695,  0., st + rt, T ],
                [ 1.,      0.8054,  -0.14695,  0., st + rt, T ]]]

plan.n_col = int(np.round(T/dt, 2))
plan.dt_arr = plan.n_col*[dt,]

#  dynamic optimization params
plan.W_X =        np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+3, 1e+4])
plan.W_X_ter = 10*np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+4, 1e+4])
plan.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
plan.rho = 5e+4

plan.X_nom = [[0.5, 0, 0.2, 0, 0, 0, 0, 0.2, 0., 0, st],
              [0.5, 0, 0.5, 0, 0, 0, 0, 0.8, 0., st, st+rt],
              [0.5, 0, 0.2, 0, 0, 0, 0, 0.0, 0., st+rt, T]]

# ik optimization params

plan.cent_wt = [1, 1e2]
plan.cnt_wt = 1e4
plan.swing_wt = None # no via points in this motion

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])
x_reg1[2] = 0.3

x_reg2 = x_reg1.copy()
x_reg2[3:7] = [0,1,0,0]
x_reg2[7:13] = 2 * [0.0, -np.pi + 0.8, -1.6]
x_reg2[13:19] = 2 * [0.0, -np.pi - 0.8, 1.6]

state_wt_1 = np.array([1e2, 0, 100] + [100, 0, 100] + 4*[1e3, 50.0, 20] \
                     + [0.00] * 3 + [10, 10, 10] + [3.5] *(rmodel.nv - 6))

state_wt_2 = np.array([1e2, 0, 1000.0] + [100, 100, 100] + 4*[1e3, 1e2, 50] \
                    + [0.00] * 3 + [10, 10, 10] + [3.5] *(rmodel.nv - 6))

plan.state_reg = [np.hstack((x_reg1, [0, st+rt])), np.hstack((x_reg2, [st+rt, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, st+rt])), np.hstack((state_wt_2, [st+rt, T]))]
plan.state_scale = [[1e-2, 0, st+rt], [500*1e-2, st+rt, T]]

ctrl_wt = [0, 0, 10] + [1, 1, 1] + [50.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[7e-5, 0, T]]