## This file contains the motion plan for caflight_timewheel
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
plan = ACyclicMotionParams("solo12", "jump_fwd")

st = 0.4
flight_time = 0.3
T = 1.2
dt = 5e-2
dt = T/30

plan.n_col = 25
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[0.3, 0, st + flight_time],
                  [0.5, st + flight_time, T]]

plan.cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                  [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                  [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                  [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                 [[ 0.,      0.3946,   0.14695,  0.,st, st + flight_time   ],
                  [ 0.,      0.3946,  -0.14695,  0., st, st + flight_time   ],
                  [ 0.,      0.0054,   0.14695,  0., st, st + flight_time   ],
                  [ 0.,      0.0054,  -0.14695,  0., st, st + flight_time   ]],

                 [[ 1.,      0.3946,   0.14695,  0.,st + flight_time, T ],
                  [ 1.,      0.3946,  -0.14695,  0., st + flight_time, T ],
                  [ 1.,      0.0054,   0.14695,  0., st + flight_time, T ],
                  [ 1.,      0.0054,  -0.14695,  0., st + flight_time, T ]]]


#  dynamic optimization params
plan.W_X = np.array([1e-5, 1e-5, 1e+5, 1e-4, 1e-4, 1e-4, 3e+4, 3e+4, 3e+4])
plan.W_X_ter = 10*np.array([1e-5, 1e-5, 1e5, 1e+2, 1e+1, 1e+2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(4*[5e+0, 5e+0, 7e+0])
plan.rho = 7e+4

plan.X_nom = [[0.2, 0, 0.22, 0, 0, 0, 0, 0.00, 0.0, 0.0, st],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st, st+flight_time],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st+flight_time, T]]

plan.X_ter = [0.2, 0, 0.2, 0, 0, 0, 0, 0.0, 0.0]

plan.bounds = [[-0.25, -0.25, 0.1, 0.25, 0.25, 0.25, 0, st],
               [-0.25, -0.25, 0.1, 0.25, 0.25, 0.3, st, T]]

# ik optimization params

plan.cent_wt = [3*[5e+1,], 6*[1e3,]]
plan.cnt_wt = 5e4

plan.swing_wt = [[[1e2, 0.3946,   0.14695,  0.0, st + 0.25*flight_time, st + 0.5*flight_time],
                  [1e2, 0.3946,   -0.14695,  0.0, st + 0.25*flight_time, st + 0.5*flight_time],
                  [1e2,  0.0054,   0.14695,  0., st + 0.25*flight_time, st + 0.5*flight_time],
                  [1e2,  0.0054,   -0.14695,  0., st + 0.25*flight_time, st + 0.5*flight_time]]]

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e-2] + [5.0, 5.0, 1e-2] + 4*[1e1, 1e-1, 1e-1] +
                      [0.00, 0.00, 1e-5] + [5.0, 5.0, 5.0] + 4*[3.5, 3.5, 3.5]
                      )

state_wt_2 = np.array([1e-2, 1e-2, 1e2] + [1e2, 1e2, 1e2] + 4*[1e1, 1e+2, 1e+2] +
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + 4*[3.5, 3.5, 3.5]
                      )

plan.state_reg = [np.hstack((x_reg1, [0, st+flight_time])), np.hstack((x_reg1, [st+flight_time, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, st+flight_time])), np.hstack((state_wt_2, [st+flight_time, T]))]
plan.state_scale = [[1e-3, 0, st+flight_time], [1e-3, st+flight_time, T]]

ctrl_wt = [0, 0, 0] + [1e-4, 1e-4, 1e-4] + [1e-4]*(rmodel.nv-6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, st+flight_time])),np.hstack((ctrl_wt, [st+flight_time, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, st+flight_time])),
                 np.hstack((np.zeros(rmodel.nv), [st+flight_time, T]))]
plan.ctrl_scale = [[1e-4, 0, st+flight_time], [1e-2, st+flight_time, T]]

# controller details
plan.kp = [[0.0, 0, st+flight_time], [2.5, st+flight_time, T]]
plan.kd = [[0.01, 0, st+flight_time], [0.1, st+flight_time, T]]
