## This file contains the motion plan for cartwheel
## Author : Avadesh Meduri
## Date : 21/09/2021

import numpy as np
from robot_properties_solo.config import Solo12Config
from motions.weight_abstract import ACyclicMotionParams
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

st = 0.4
flip_time = 0.5
T = 1.2
dt = 5e-2

plan.cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                [[ 1.,      0.3946,   0.14695,  0.,st, st + flip_time],
                [ 1.,      0.3946,  -0.14695,  0., st, st + flip_time],
                [ 0.,      0.0054,   0.14695,  0., st, st + flip_time],
                [ 0.,      0.0054,  -0.14695,  0., st, st + flip_time]],

                [[ 1.,      0.3946,   0.14695,  0.,st + flip_time, T ],
                [ 1.,      0.3946,  -0.14695,  0., st + flip_time, T ],
                [ 1.,      0.8054,   0.14695,  0., st + flip_time, T ],
                [ 1.,      0.8054,  -0.14695,  0., st + flip_time, T ]]]

plan.n_col = int(np.round(T/dt))
plan.dt_arr = (plan.n_col+1)*[dt,]
plan.plan_freq = [[0.6, 0, T],
                  [1.0, T, T+1.5]]
#  dynamic optimization params
plan.W_X =        np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+3, 1e+4])
plan.W_X_ter = 10*np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+4, 1e+4])
plan.W_F = np.array(4*[1e+1, 1e+1, 2e+0])
plan.rho = 5e+4

plan.X_nom = [[0.2, 0, 0.2, 0, 0, 0, 0, 0.1, 0., 0, st],
              [0.4, 0, 0.3, 0, 0, 0, 0, 0.6, 0., st, st+flip_time],
              [0.6, 0, 0.2, 0, 0, 0, 0, 0.0, 0., st+flip_time, T]]

plan.X_ter = [0.2, 0, 0.2, 0, 0, 0, 0, 0.0, 0.0]

plan.bounds = [[-0.45, -0.45, 0.0, 0.45, 0.45, 0.3, 0, st],
               [-0.45, -0.45, 0.0, 0.45, 0.45, 0.45, st, T]]

# ik optimization params

plan.cent_wt = [1, 3e3]
plan.cnt_wt = 1e4
plan.swing_wt = None # no via points in this motion


# plan.swing_wt = [[[1e2, 0.4,   0.14695,  0.6, st + 0.25*rear_time, st + 0.5*rear_time],
#                   [1e2, 0.4,   -0.14695,  0.3, st + 0.25*rear_time, st + rear_time],
#                   [0,  0.0054,   0.14695,  0., st + 0.25*rear_time, st + 0.5*rear_time],
#                   [0,  0.0054,   -0.14695,  0., st + 0.25*rear_time, st + 0.5*rear_time]]]


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

plan.state_reg = [np.hstack((x_reg1, [0, st+flip_time])), np.hstack((x_reg2, [st+flip_time, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, st+flip_time])), np.hstack((state_wt_2, [st+flip_time, T]))]
plan.state_scale = [[1e-2, 0, st+flip_time], [500*1e-2, st+flip_time, T]]

ctrl_wt = [0, 0, 10] + [1, 1, 1] + [70.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[7e-4, 0, T]]

# controller details 
plan.kp = [[5.5, 0, T]]
plan.kd = [[0.1, 0, T]]