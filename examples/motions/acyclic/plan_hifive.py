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
plan = ACyclicMotionParams("solo12", "hifive")

st = 0.5
rear_time = 0.4
jt = 0.10
T = 1.4
dt = 0.05

plan.cnt_plan = [[[1.,      0.3946,   0.14695,  0., 0.,  st    ],
                  [1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                  [1.,      0.0054,   0.14695,  0., 0.,  st    ],
                  [1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                 [[0.,      0.3946,   0.14695,  0., st, st + rear_time],
                  [0.,      0.3946,  -0.14695,  0., st, st + rear_time],
                  [1.,      0.0054,   0.14695,  0., st, st + rear_time],
                  [1.,      0.0054,  -0.14695,  0., st, st + rear_time]],

                 [[0.,      0.3946,   0.14695,  0., st+rear_time, st + rear_time + jt],
                  [0.,      0.3946,  -0.14695,  0., st+rear_time, st + rear_time + jt],
                  [0.,      0.0054,   0.14695,  0., st+rear_time, st + rear_time + jt],
                  [0.,      0.0054,  -0.14695,  0., st+rear_time, st + rear_time + jt]],

                 [[1.,      0.41,   0.14695,  0.,   st + rear_time + jt, T],
                  [1.,      0.41,  -0.14695,  0.,   st + rear_time + jt, T],
                  [1.,      -0.0054,   0.14695,  0., st + rear_time + jt, T],
                  [1.,      -0.0054,  -0.14695,  0., st + rear_time + jt, T]]]

plan.n_col = 25
plan.dt_arr = plan.n_col*[dt, ]
plan.plan_freq = [[1.4, 0, st ],
                  [1.4, st, st + rear_time+jt],
                  [0.05, st + rear_time+jt, T]]

#  dynamic optimization params
plan.W_X =        np.array([1e+3, 1e1, 1e+2, 1e-4, 1e-4, 1e-4, 1e+2, 5e+3, 1e+2])
plan.W_X_ter = 10*np.array([1e+3, 1e1, 1e+5, 1e-1, 1e-1, 1e-1, 1e+2, 1e+4, 1e+2])
plan.W_F = np.array(4*[1e+1, 1e+1, 5e-1])
plan.rho = 5e+4

plan.X_nom = [[0.2, 0, 0.22, 0, 0, 0, 0, -0.05, 0.0, 0.0, st],
              [0.18, 0, 0.28, 0, 0, 0, 0, -0.45, 0., st, st+rear_time],
              [0.18, 0, 0.32, 0, 0, 0, 0, 0.0, 0., st+rear_time, st+rear_time+jt],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st+rear_time+jt, T]]

plan.X_ter = [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0.0]

plan.bounds = [ [-0.25, -0.25, 0.1, 0.25, 0.25, 0.25, 0, st],
                [-0.25, -0.25, 0.1, 0.25, 0.25, 0.4, st, st+rear_time],
                [-0.25, -0.25, 0.1, 0.25, 0.25, 0.25, st+rear_time, T]]

# ik optimization params

plan.cent_wt = [3*[1e1,], 6*[1e2,]]  # CoM, Momentum
plan.cnt_wt = 1e4  # End-Effector Contact Weight

plan.swing_wt = [[[0e2, 0.55,   0.14695,  0.7, st + 0.4*rear_time, st + 0.5*rear_time],
                  [0e2, 0.55,   -0.14695,  0.7, st + 0.4*rear_time, st + 0.5*rear_time],
                  [0,  0.0054,   0.14695,  0., st + 0.25*rear_time, st + 0.5*rear_time],
                  [0,  0.0054,   -0.14695,  0., st + 0.25*rear_time, st + 0.5*rear_time]]]

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])
x_reg2 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e-2] + [0, 0, 1.0] + 2*[1e-3, 1e-3, 1e-3] + 2*[1e1, 5e1, 5e1] +
                      3*[0.00] + [0, 0, 1.0] + (rmodel.nv - 6)*[0.5])

state_wt_2 = np.array([1e-2, 1e-2, 1e2] + [1e2, 1e2, 1e2] + 4*[1e1, 1e+2, 1e+2] +
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + 4*[3.5, 3.5, 3.5]
                      )

plan.state_reg = [np.hstack((x_reg1, [0, st + rear_time])), np.hstack((x_reg1, [st + rear_time, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, st + rear_time])), np.hstack((state_wt_2, [st + rear_time, T]))]
plan.state_scale = [[1e-2, 0, st + rear_time], [1e-2, st + rear_time, T]]

ctrl_wt = [0, 0, 10] + [1, 1, 1] + [10.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[5e-4, 0, T]]

# controller details
plan.kp = [[2.0, 0, T]]
plan.kd = [[0.1, 0, T]]