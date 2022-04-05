## This file contains the motion plan for Standing In Place
## Author : Avadesh Meduri & Paarth Shah
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
plan.n_col = 30
dt = T/30
plan.dt_arr = plan.n_col*[dt,]

plan.cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                  [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                  [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                  [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

                 [[ 1.,      0.3946,   0.14695,  0.,st, st + flight_time   ],
                  [ 1.,      0.3946,  -0.14695,  0., st, st + flight_time   ],
                  [ 1.,      0.0054,   0.14695,  0., st, st + flight_time   ],
                  [ 1.,      0.0054,  -0.14695,  0., st, st + flight_time   ]],

                 [[ 1.,      0.3946,   0.14695,  0.,st + flight_time, T ],
                  [ 1.,      0.3946,  -0.14695,  0., st + flight_time, T ],
                  [ 1.,      0.0054,   0.14695,  0., st + flight_time, T ],
                  [ 1.,      0.0054,  -0.14695,  0., st + flight_time, T ]]]


#  dynamic optimization params
plan.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e-4, 1e-4, 2e2, 3e+4, 3e+4, 3e+4])
plan.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
plan.rho = 5e+4

plan.X_nom = [[0.2, 0, 0.22, 0, 0, 0, 0, 0.00, 0.0, 0.0, st],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st, st+flight_time],
              [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0., st+flight_time, T]]

plan.X_ter = [0.2, 0, 0.22, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[0.25, 0.25, 0.25, 0, T]]

# ik optimization params

plan.cent_wt = [1e1, 1e4]
plan.cnt_wt = 1e1

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + 4*[1e2, 1e2, 1e2] + \
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + 4*[3.5, 3.5, 3.5]
                      )

plan.state_reg = [np.hstack((x_reg1, [0, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, T]))]
plan.state_scale = [[1e-2, 0, T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[1e-4, 0, T]]

# controller details
plan.kp = 2.5
plan.kd = 0.5
