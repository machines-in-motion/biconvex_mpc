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

v0 = pin.utils.zero(pin_robot.model.nv)
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
