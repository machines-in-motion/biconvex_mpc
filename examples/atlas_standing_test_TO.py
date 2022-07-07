## This is a demo for atlas
## Author : Avadesh Meduri
## Date : 06/04/2022

import time
import numpy as np
from mpc.abstract_cyclic_gen1 import AbstractGaitGen
from robot_properties_atlas.config import AtlasConfig

import pinocchio as pin

import numpy as np
from motions.cyclic.atlas_stand import still

robot = AtlasConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data
viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)

## robot config and init
pin_robot = AtlasConfig.buildRobotWrapper()
urdf_path = AtlasConfig.urdf_path


eff_names = ["l_foot_lt", "l_foot_rt", "l_foot_lb", "l_foot_rb", "r_foot_lt", "r_foot_rt", "r_foot_lb", "r_foot_rb"]
hip_names = ["l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz"]
n_eff = len(eff_names)

q0 = np.array(AtlasConfig.initial_configuration)
q0[0:2] = 0.0
# q0 = np.array([ 0.00000000e+00,  0.00000000e+00,  8.62122297e-01, -1.70664498e-04,
#                -9.88964573e-04, -3.15211568e-04,  9.99999464e-01, -1.52856635e-03,
#                 4.66264401e-03,  1.09217098e-03,  4.75506438e-03, -1.18797149e+00,
#                -3.04919881e-02, -4.99176179e-08,  3.73394711e-02,  3.33899279e-03,
#                 4.32833113e-02, -1.82226696e-02,  1.21055938e+00, -1.95533406e-02,
#                 1.97581115e-04,  4.78171936e-02, -1.55972771e-03, -3.63221700e-02,
#                 5.19289134e-04,  2.11599179e-04, -5.00127616e-01,  1.00290045e+00,
#                -5.00791338e-01,  1.36663149e-04,  7.33373111e-04,  1.97467693e-04,
#                -5.00087295e-01,  1.00255771e+00, -5.00489869e-01,  1.55055706e-04])

v0 = np.array( [-1.74551400e-02,  6.04884025e-03, -2.05654376e-02, -2.65868301e-02,
                -1.97143050e-01, -1.95181605e-02, -2.68233642e-02,  3.09252186e-01,
                 5.76636422e-02, -6.72528679e-01,  1.99106263e-01, -3.54850840e+01,
                 1.25644316e-05,  1.00000000e+02,  1.00306034e+00,  1.00000000e+02,
                 2.60032178e-01,  1.11251022e-01, -3.49227667e+01, -5.17000612e-02,
                 1.00000000e+02,  2.26576334e+00, -1.00000000e+02,  1.94548214e-02,
                 1.96340038e-02,  1.54666604e-01,  9.00458069e-02, -4.75667724e-02,
                 7.13372800e-03,  1.94569323e-02,  1.97082914e-02,  1.64211513e-01,
                 6.20027361e-02, -2.90750268e-02,  6.67676346e-03])

# v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0,0.0])
w_des = 0.0

update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = 0.05
index = 0
pln_ctr = 0
q = q0
v = v0
plan_freq = still.gait_horizon * still.gait_period # sec


gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)
gg.update_gait_params(still, 0)

q = q0
v = v0
step_t = 0
n = 0

xs_plan, us_plan, f_plan = gg.optimize(q, v, 0., v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
gg.plot(q, v, plot_force=True)
