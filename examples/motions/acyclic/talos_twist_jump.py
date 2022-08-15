## This file contains the motion plan for walking of Talos
## Author : Majid Khadiv
## Date : 22/07/2022

import numpy as np
from robot_properties_talos.config import TalosConfig
from motions.weight_abstract import ACyclicMotionParams
import pinocchio as pin
###########################################################################################

robot = TalosConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

q0 = np.array(TalosConfig.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

###########################################################################################
plan = ACyclicMotionParams("Talos", "walk")

stance_time = 0.4 #Double support time
flight_time = 0.4 #Single support time

HC_init_x = -0.089 #the initial position of the heel contact point in x direction
TC_init_x = .131 #the initial position of the Toe contact point in x direction
MC_init_y = .085 #the initial position of the contact points in y direction, points are symmetric in y
plan.T = 2*stance_time+flight_time # the total duration of the gait
jump_length = .5 # the length of jump forward
plan.dt = 2e-2
plan.n_col = int(plan.T/plan.dt)
plan.dt_arr = plan.n_col*[plan.dt,]
plan.plan_freq = [[plan.T, 0., plan.T]]
# replanning around offline trajectory
plan.use_offline_traj = True

plan.cnt_plan = [[[ 1., HC_init_x, -MC_init_y,  0., 0.,  stance_time],
                  [ 1., TC_init_x, -MC_init_y,  0., 0.,  stance_time],
                  [ 1., HC_init_x,  MC_init_y,  0., 0.,  stance_time],
                  [ 1., TC_init_x,  MC_init_y,  0., 0.,  stance_time]],

                 [[ 0., HC_init_x, -MC_init_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x, -MC_init_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., HC_init_x,  MC_init_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x,  MC_init_y,  0., stance_time,  stance_time+flight_time]],

                 [[ 1., -MC_init_y, -HC_init_x,  0., stance_time+flight_time,  plan.T],
                  [ 1., -MC_init_y, -TC_init_x,  0., stance_time+flight_time,  plan.T],
                  [ 1.,  MC_init_y, -HC_init_x,  0., stance_time+flight_time,  plan.T],
                  [ 1.,  MC_init_y, -TC_init_x,  0., stance_time+flight_time,  plan.T]]]
#  dynamic optimization params
plan.W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 2e2, 3e+4, 3e+1, 3e+4])
plan.W_X_ter = 10.*np.array([1e5, 1e5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(8*[1e+1, 1e+1, 1e+1])
plan.rho = 1e+4

plan.X_nom = [[jump_length/3., 0, 0.4, 0, 0, 0, 0, 0.00, 0.0, 0.0, stance_time],
              [jump_length, 0, 1.2, 0, 0, 0, 0, 0.00, 0.0, stance_time, stance_time+flight_time],
              [jump_length+.2, 0., 0.9, 0, 0, 0, 0, 0.0, 0., stance_time+flight_time, plan.T]]

plan.X_ter = [jump_length, 0., 0.9, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.5, -0.5, .3, 0.5, 0.5, 2., 0., plan.T]]

# ik optimization params
plan.swing_wt = [[[0., 0., -MC_init_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0., -MC_init_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time]]]
plan.cent_wt = [3*[100.,], [.04, .04, .04,.4, .4, .4]]
plan.cnt_wt = 5e3

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 10.0, 5.0] + [1e1] * (robot.model.nv - 6) + \
                      [0.00, 0.00, 0.00] + [5.0, 10.0, 5.0] + [3.] * (robot.model.nv - 6)
                      )

plan.state_reg = [np.hstack((x_reg1, [0, plan.T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, plan.T]))]
plan.state_scale = [[2e-1, 0, plan.T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [.1] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, plan.T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, plan.T]))]
plan.ctrl_scale = [[1e-4, 0, plan.T]]

# controller details
plan.kp = [[100., 0., stance_time],
           [100., stance_time, stance_time+flight_time],
           [100., stance_time+flight_time, plan.T]]
plan.kd = [[5., 0., stance_time+flight_time],
           [5., stance_time, stance_time+flight_time],
           [5., stance_time+flight_time, plan.T]]
