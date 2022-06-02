## This file contains the motion plan for Standing In Place
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/09/2021

import numpy as np
from robot_properties_bolt.config import BoltHumanoidConfig
from motions.weight_abstract import ACyclicMotionParams
import pinocchio as pin
###########################################################################################

robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

q0 = np.array(BoltHumanoidConfig.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])


###########################################################################################
plan = ACyclicMotionParams("bolt_humanoid", "stand")

T = 10
dt = 5e-2
plan.n_col = 30
# dt = T/30
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[0.1, 0, T]]

plan.cnt_plan = [[[ 1.,      0.0,   0.1235,  0.03343, 0.,  T    ],
                  [ 1.,      0.0,  -0.1235,  0.03343, 0.,  T    ],
                  [ 0.,      0.0,   0.1235,  0.03343, 0.,  T    ],
                  [ 0.,      0.0,  -0.1235,  0.03343, 0.,  T    ]]]


#  dynamic optimization params
plan.W_X =        np.array([1e0, 1e0, 1e+0, 1e-4, 1e-4, 2e0, 3e+4, 1e+4, 3e+4])
plan.W_X_ter = 10.*np.array([1e3, 1e-5, 1e+0, 1e-1, 1e-1, 2e2, 1e+1, 1e+1, 1e+1])
plan.W_F = 0.*np.array(2*[1e+1, 1e+1, 1e+1] + 2*[0., 0., 0.])
plan.rho = 5e+4

plan.X_nom = [[0., 0, 0.4096, 0, 0, 0, 0, 0.00, 0.0, 0.0, T]]

plan.X_ter = [0., 0., 0.4096, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.25, -0.25, 0.2, 0.25, 0.25, 0.5, 0., T]]

# ik optimization params

plan.cent_wt = [1e1, 1e4]
plan.cnt_wt = 1e1

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + 2*[1e2, 1e2, 1e2] + [1e2, 1e2, 1e2] +\
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + 2*[3.5, 3.5, 3.5] + [.1, .1, .1]
                      )

plan.state_reg = [np.hstack((x_reg1, [0, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, T]))]
plan.state_scale = [[1e-2, 0, T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[1e-4, 0, T]]

# controller details
plan.kp = [[2.5, 0, T]]
plan.kd = [[0.5, 0, T]]
