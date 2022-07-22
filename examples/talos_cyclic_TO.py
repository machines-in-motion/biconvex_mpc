## This is a demo for atlas
## Author : Avadesh Meduri
## Date : 06/04/2022

import time
import numpy as np
from robot_properties_talos.config import TalosConfig
from robot_properties_talos.taloswrapper import TalosRobot
from mpc.abstract_cyclic_gen1 import AbstractGaitGen

from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv

import pinocchio as pin

import numpy as np
from motions.cyclic.talos_stand import still
from motions.cyclic.talos_walk import walk


robot = TalosConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

## robot config and init
pin_robot = TalosConfig.buildRobotWrapper()
urdf_path = TalosConfig.urdf_path

# viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
# viz.initViewer(open=False)
# viz.loadViewerModel()

eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint"]
hip_names = ["leg_left_1_joint", "leg_left_1_joint", "leg_right_1_joint", "leg_right_1_joint"]
n_eff = len(eff_names)

q0 = np.array(TalosConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.5,0.0,0.0])
w_des = 0.0

plan_freq = 0.1 # sec
update_time = 0.0 # sec (time of lag)

gait_params = walk


robot_id_ctrl = InverseDynamicsController(pin_robot, eff_names)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)
gg.update_gait_params(gait_params, 0)

q = q0
v = v0

# simulation variables
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
lag = 0

# robot.start_recording("talos_walk.mp4")
plot_time = np.inf

# try:
#     viz.initViewer(open=True)
# except ImportError as err:
#     print(
#         "Error while initializing the viewer. It seems you should install Python meshcat"
#     )
#     print(err)
#     sys.exit(0)
#
# viz.loadViewerModel()
# viz.display(q0)

v_des = np.array([0.0, 0.0, 0.0])
w_des = 0.0
xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
gg.plot(q, v, plot_force=True)
# print(np.size(xs_plan[:][:pin_robot.model.nq]))

# for ind in range(5151):
#     print(ind)
#     viz.display(xs_plan[ind][:pin_robot.model.nq])
