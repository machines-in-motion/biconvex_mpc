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

DSP_time = 0.2 #Double support time
SSP_time = 0.8 #Single support time
step_length = 0.7
HC_init_x = -.109 #the initial position of the heel contact point in x direction
TC_init_x = .111 #the initial position of the Toe contact point in x direction
MC_init_in_y = .035 #the initial position of the contact points in y direction, the inward edge
MC_init_out_y = .135 #the initial position of the contact points in y direction, the outward edge
plan.T = 2.2 # the total duration of the gait
dt = 1e-2
plan.n_col = int(plan.T/dt)
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[plan.T, 0, plan.T]]

plan.cnt_plan = [[[ 1., HC_init_x, -MC_init_out_y,  0., 0.,  DSP_time],
                  [ 1., TC_init_x, -MC_init_out_y,  0., 0.,  DSP_time],
                  [ 1., HC_init_x, -MC_init_in_y ,  0., 0.,  DSP_time],
                  [ 1., TC_init_x, -MC_init_in_y ,  0., 0.,  DSP_time],
                  [ 1., HC_init_x,  MC_init_in_y ,  0., 0.,  DSP_time],
                  [ 1., TC_init_x,  MC_init_in_y ,  0., 0.,  DSP_time],
                  [ 1., HC_init_x,  MC_init_out_y ,  0., 0.,  DSP_time],
                  [ 1., TC_init_x,  MC_init_out_y ,  0., 0.,  DSP_time]],

                 [[ 0., HC_init_x, -MC_init_out_y,  0., DSP_time,  DSP_time + SSP_time],
                  [ 0., TC_init_x, -MC_init_out_y,  0., DSP_time,  DSP_time + SSP_time],
                  [ 0., HC_init_x, -MC_init_in_y ,  0., DSP_time,  DSP_time + SSP_time],
                  [ 0., TC_init_x, -MC_init_in_y ,  0., DSP_time,  DSP_time + SSP_time],
                  [ 1., HC_init_x,  MC_init_in_y ,  0., DSP_time,  DSP_time + SSP_time],
                  [ 1., TC_init_x,  MC_init_in_y ,  0., DSP_time,  DSP_time + SSP_time],
                  [ 1., HC_init_x,  MC_init_out_y ,  0., DSP_time,  DSP_time + SSP_time],
                  [ 1., TC_init_x,  MC_init_out_y ,  0., DSP_time,  DSP_time + SSP_time]],

                  [[ 1., HC_init_x + step_length, -MC_init_out_y,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., TC_init_x + step_length, -MC_init_out_y,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., HC_init_x + step_length, -MC_init_in_y ,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., TC_init_x + step_length, -MC_init_in_y ,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., HC_init_x,  MC_init_in_y ,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., TC_init_x,  MC_init_in_y ,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., HC_init_x,  MC_init_out_y,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time],
                   [ 1., TC_init_x,  MC_init_out_y,  0., DSP_time + SSP_time,  2*DSP_time + SSP_time]],

                   [[ 1., HC_init_x + step_length, -MC_init_out_y,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 1., TC_init_x + step_length, -MC_init_out_y,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 1., HC_init_x + step_length, -MC_init_in_y ,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 1., TC_init_x + step_length, -MC_init_in_y ,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 0., HC_init_x,  MC_init_in_y ,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 0., TC_init_x,  MC_init_in_y ,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 0., HC_init_x,  MC_init_out_y,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)],
                    [ 0., TC_init_x,  MC_init_out_y,  0., 2*DSP_time + SSP_time,  2*(DSP_time + SSP_time)]],

                    [[ 1., HC_init_x + step_length, -MC_init_out_y,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., TC_init_x + step_length, -MC_init_out_y,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., HC_init_x + step_length, -MC_init_in_y ,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., TC_init_x + step_length, -MC_init_in_y ,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., HC_init_x + step_length,  MC_init_in_y ,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., TC_init_x + step_length,  MC_init_in_y ,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., HC_init_x + step_length,  MC_init_out_y,  0., 2*(DSP_time + SSP_time),  plan.T],
                     [ 1., TC_init_x + step_length,  MC_init_out_y,  0., 2*(DSP_time + SSP_time),  plan.T]]]



#  dynamic optimization params
plan.W_X = np.array([1e-5, 1e-5, 1e+5, 1e-4, 1e-4, 2e2, 3e+4, 3e+4, 3e+4])
plan.W_X_ter = 10.*np.array([1e5, 1e5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
plan.rho = 1e+4

plan.X_nom = [[0., 0, 0.9, 0, 0, 0, 0, 0.00, 0.0, 0.0, DSP_time],
              [step_length/2., 0., 0.9, 0, 0, 0, 0, 0.0, 0., DSP_time, DSP_time+SSP_time],
              [step_length, 0., 0.9, 0, 0, 0, 0, 0.0, 0., DSP_time+SSP_time, plan.T]]

plan.X_ter = [step_length, 0., 0.9, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.5, -0.5, .6, 0.5, 0.5, 1.2, 0., plan.T]]

# ik optimization params
plan.swing_wt = [[[10., step_length/2, -MC_init_out_y,  0.2, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [10., step_length/2, -MC_init_out_y,  0.2, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [10., step_length/2, -MC_init_in_y,  0.2, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [10., step_length/2, -MC_init_in_y,  0.2, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [0., step_length/2,  MC_init_in_y,  0.0, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [0., step_length/2,  MC_init_in_y,  0.0, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [0., step_length/2,  MC_init_out_y,  0.0, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time],
                  [0., step_length/2,  MC_init_out_y,  0.0, DSP_time+.6*SSP_time, DSP_time+.7*SSP_time]],

                 [[0., step_length/2, -MC_init_out_y,  0.0, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0., step_length/2, -MC_init_out_y,  0.0, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0., step_length/2, -MC_init_in_y,  0.0, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0., step_length/2, -MC_init_in_y,  0.0, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0.1, step_length/2,  MC_init_in_y, 0.2, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0.1, step_length/2,  MC_init_in_y, 0.2, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0.1, step_length/2,  MC_init_out_y, 0.2, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time],
                  [0.1, step_length/2,  MC_init_out_y, 0.2, 2*DSP_time+1.4*SSP_time, 2*DSP_time+1.5*SSP_time]]]
plan.cent_wt = [3*[100.,], 6*[.05,]]
plan.cnt_wt = 5e3

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + [1e1] * (robot.model.nv - 6) + \
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + [3.5] * (robot.model.nv - 6)
                      )

plan.state_reg = [np.hstack((x_reg1, [0, plan.T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, plan.T]))]
plan.state_scale = [[1e-2, 0, plan.T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, plan.T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, plan.T]))]
plan.ctrl_scale = [[1e-4, 0, plan.T]]

# controller details
plan.kp = [[100, 0., plan.T]]
plan.kd = [[5., 0., plan.T]]
