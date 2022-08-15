## This file contains the motion plan for Standing In Place
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/09/2021

import numpy as np
from robot_properties_atlas.config import AtlasConfig
from motions.weight_abstract import ACyclicMotionParams
import pinocchio as pin
###########################################################################################

robot = AtlasConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

q0 = np.array(AtlasConfig.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])


###########################################################################################
plan = ACyclicMotionParams("Atlas", "stand")

stance_time = 0.4 #Double support time
flight_time = 0.4 #Single support time

HC_init_x = -.130 #the initial position of the heel contact point in x direction
TC_init_x = .097 #the initial position of the Toe contact point in x direction
MC_init_in_y = .045 #the initial position of the contact points in y direction, the inward edge
MC_init_out_y = .178 #the initial position of the contact points in y direction, the outward edge
plan.T = 2*stance_time+flight_time # the total duration of the gait
dt = 2e-2
plan.n_col = int(plan.T/dt)
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[plan.T, 0., plan.T]]

plan.cnt_plan = [[[ 1., HC_init_x, MC_init_out_y,  0., 0.,  stance_time],
                  [ 1., TC_init_x, MC_init_out_y,  0., 0.,  stance_time],
                  [ 1., HC_init_x, MC_init_in_y ,  0., 0.,  stance_time],
                  [ 1., TC_init_x, MC_init_in_y ,  0., 0.,  stance_time],
                  [ 1., HC_init_x, -MC_init_in_y ,  0., 0.,  stance_time],
                  [ 1., TC_init_x, -MC_init_in_y ,  0., 0.,  stance_time],
                  [ 1., HC_init_x, -MC_init_out_y ,  0., 0.,  stance_time],
                  [ 1., TC_init_x, -MC_init_out_y ,  0., 0.,  stance_time]],

                 [[ 0., HC_init_x,  MC_init_out_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x,  MC_init_out_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., HC_init_x,  MC_init_in_y ,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x,  MC_init_in_y ,  0., stance_time,  stance_time+flight_time],
                  [ 0., HC_init_x, -MC_init_in_y ,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x, -MC_init_in_y ,  0., stance_time,  stance_time+flight_time],
                  [ 0., HC_init_x, -MC_init_out_y,  0., stance_time,  stance_time+flight_time],
                  [ 0., TC_init_x, -MC_init_out_y,  0., stance_time,  stance_time+flight_time]],

                 [[ 1., HC_init_x,  MC_init_out_y,  0., stance_time+flight_time,  plan.T],
                  [ 1., TC_init_x,  MC_init_out_y,  0., stance_time+flight_time,  plan.T],
                  [ 1., HC_init_x,  MC_init_in_y ,  0., stance_time+flight_time,  plan.T],
                  [ 1., TC_init_x,  MC_init_in_y ,  0., stance_time+flight_time,  plan.T],
                  [ 1., HC_init_x, -MC_init_in_y ,  0., stance_time+flight_time,  plan.T],
                  [ 1., TC_init_x, -MC_init_in_y ,  0., stance_time+flight_time,  plan.T],
                  [ 1., HC_init_x, -MC_init_out_y,  0., stance_time+flight_time,  plan.T],
                  [ 1., TC_init_x, -MC_init_out_y,  0., stance_time+flight_time,  plan.T]]]


#  dynamic optimization params
plan.W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 2e2, 3e+4, 3e+4, 3e+4])
plan.W_X_ter = 10.*np.array([1e5, 1e5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(8*[1e+1, 1e+1, 1e+1])
plan.rho = 1e+4

plan.X_nom = [[0., 0, 0.4, 0, 0, 0, 0, 0.00, 0.0, 0.0, stance_time],
              [0., 0, 1.2, 0, 0, 0, 0, 0.00, 0.0, stance_time, stance_time+flight_time],
              [0., 0., 0.9, 0, 0, 0, 0, 0.0, 0., stance_time+flight_time, plan.T]]

plan.X_ter = [0., 0., 0.9, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.5, -0.5, .3, 0.5, 0.5, 2., 0., plan.T]]

# ik optimization params
plan.swing_wt = [[[0., 0., -MC_init_out_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0., -MC_init_out_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0., -MC_init_in_y ,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0., -MC_init_in_y ,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_out_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_out_y,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_in_y ,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time],
                  [0., 0.,  MC_init_in_y ,  0.5, stance_time+.5*flight_time, stance_time+.6*flight_time]]]

plan.cent_wt = [3*[50.,], 6*[.04,]]
plan.cnt_wt = 5e3

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])
print(x_reg1)
print(q0)
state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + [1e1] * (robot.model.nv - 6) + \
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + [3.] * (robot.model.nv - 6)
                      )

plan.state_reg = [np.hstack((x_reg1, [0, plan.T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, plan.T]))]
plan.state_scale = [[8e-3, 0, plan.T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [.1] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, plan.T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, plan.T]))]
plan.ctrl_scale = [[1e-4, 0, plan.T]]

# controller details
plan.kp = [[20., 0., plan.T]]
plan.kd = [[1., 0., plan.T]]
