## This is a demo for atlas
## Author : Avadesh Meduri
## Date : 06/04/2022

import time
import numpy as np
from mpc.abstract_cyclic_gen1 import AbstractGaitGen
from robot_properties_atlas.config import AtlasConfig

import pinocchio as pin

import numpy as np
from motions.weight_abstract import BiconvexMotionParams

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

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0, foot_size=0.0)

#### walk #########################################
walk = BiconvexMotionParams("atlas", "walk")

# Cnt
walk.gait_period = 0.8
walk.stance_percent = n_eff*[0.6,]
walk.gait_dt = 0.01
walk.phase_offset = int(0.5*n_eff)*[0.0,] + int(0.5*n_eff)*[0.5,] 

# IK
walk.state_wt = np.array([1e2, 1e2, 1e2] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
                         + [0.00] * 3 + [100] * 3 + [.1] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)

walk.swing_wt = [1e5, 2e5]
walk.cent_wt = [5e+1, 1e+2]
walk.step_ht = 0.3
walk.nom_ht = 1.12
walk.reg_wt = [5e-2, 1e-5]

# Dyn
walk.W_X =     np.array([1e0, 1e0, 1e+4, 1e-4, 1e-4, 2e0, 3e-1, 1e-1, 3e-1])
walk.W_X_ter = 10.*np.array([1e0, 1e0, 1e+6, 1e-1, 1e-1, 2e3, 1e-1, 1e-1, 1e-1])
walk.W_F = np.array(8*[1e2, 1e2, 1e2])
walk.rho = 5e4

walk.ori_correction = [0.0, 0.0, 0.0]
walk.gait_horizon = 3

# Gains
walk.kp = 100.5
walk.kd = 5.0

gg.update_gait_params(walk, 0)

sim_t = 0.0
sim_dt = 0.05
index = 0
pln_ctr = 0
q = q0
v = v0

xs_plan, us_plan, f_plan = gg.optimize(q, v, 0., v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
gg.plot(q, v, plot_force=True)
