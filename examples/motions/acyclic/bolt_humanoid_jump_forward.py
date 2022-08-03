## This file contains the motion plan for jump motion
## Author : Majid Khadiv
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
plan = ACyclicMotionParams("bolt", "jump")


stance_time = 0.4
flight_time = 0.3

plan.T = 2*stance_time+flight_time # the total duration of the gait
dt = 2e-2
plan.n_col = int(plan.T/dt)
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[.1, 0., plan.T]]

plan.cnt_plan = [[[ 1.,      0.,   0.1235,    0., 0.,  stance_time],
                  [ 1.,      0.,  -0.1235,    0., 0.,  stance_time],
                  [ 0.,      0.23,   0.11,  0.49, 0.,  stance_time],
                  [ 0.,      0.23,  -0.11,  0.49, 0.,  stance_time]],

                 [[ 0.,      0.,   0.1235,    0., stance_time, stance_time + flight_time],
                  [ 0.,      0.,  -0.1235,    0., stance_time, stance_time + flight_time],
                  [ 0.,      0.23,   0.11,  0.49, stance_time, stance_time + flight_time],
                  [ 0.,      0.23,  -0.11,  0.49, stance_time, stance_time + flight_time]],

                 [[ 1.,      0.1,   0.1235,   0., stance_time + flight_time, plan.T],
                  [ 1.,      0.1,  -0.1235,   0., stance_time + flight_time, plan.T],
                  [ 0.,      0.23,   0.11, 0.49, stance_time + flight_time, plan.T],
                  [ 0.,      0.23,  -0.11, 0.49, stance_time + flight_time, plan.T]]]


#  dynamic optimization params
plan.W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 2e2, 3e+4, 3e+1, 3e+4])
plan.W_X_ter = 10.*np.array([1e5, 1e5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(8*[1e+1, 1e+1, 1e+1])
plan.rho = 1e+4

plan.X_nom = [[0., 0, 0.38, 0, 0, 0, 0, 0.00, 0.0, 0.0, stance_time],
              [0., 0, 0.38, 0, 0, 0, 0, 0.0, 0., stance_time, stance_time+flight_time],
              [0.1, 0, 0.38, 0, 0, 0, 0, 0.0, 0., stance_time+flight_time, plan.T]]

plan.X_ter = [0.1, 0, 0.38, 0, 0, 0, 0, 0.0, 0.0]

plan.bounds = [[-0.5, -0.5, -.5, 0.5, 0.5, 1., 0., plan.T]]

# ik optimization params

plan.cent_wt = [100., 50.]
plan.cnt_wt = 5e3

plan.swing_wt = [[[0., 0.3946,   0.14695,  0.0, stance_time + 0.25*flight_time, stance_time + 0.5*flight_time],
                  [0., 0.3946,   -0.14695,  0.0, stance_time + 0.25*flight_time, stance_time + 0.5*flight_time],
                  [0.,  0.0054,   0.14695,  0., stance_time + 0.25*flight_time, stance_time + 0.5*flight_time],
                  [0.,  0.0054,   -0.14695,  0., stance_time + 0.25*flight_time, stance_time + 0.5*flight_time]]]

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + [1e1] * (robot.model.nv - 6) + \
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + [3.] * (robot.model.nv - 6)
                      )


plan.state_reg = [np.hstack((x_reg1, [0, plan.T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, plan.T]))]
plan.state_scale = [[1e-3, 0, plan.T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [.1] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, plan.T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, plan.T]))]
plan.ctrl_scale = [[1e-4, 0, plan.T]]

# controller details
plan.kp = [[10., 0., stance_time],
           [2.5, stance_time, plan.T]]
plan.kd = [[.1, 0., stance_time],
           [.1, stance_time, plan.T]]
