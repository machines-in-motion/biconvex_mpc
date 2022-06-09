## This is a demo for cyclic motions using mpc
## Author : Majid Khadiv
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig
from mpc.abstract_cyclic_gen import BoltHumanoidMpcGaitGen
from motions.cyclic.bolt_humanoid_walk import walk
from motions.cyclic.bolt_humanoid_jump import jump

from envs.pybullet_env import PyBulletEnv
from controllers.robot_id_controller import InverseDynamicsController

## robot config and init
pin_robot = BoltHumanoidConfig.buildRobotWrapper()
urdf_path = BoltHumanoidConfig.urdf_path

viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
pin_robot = BoltHumanoidConfig.buildRobotWrapper()

n_eff = 4
q0 = np.array(BoltHumanoidConfig.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_ANKLE", "FR_ANKLE", "L_WRIST", "R_WRIST"]

v_des = np.array([-0.0, 0.0, 0.0])
w_des = 0.0

plan_freq = 5 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

## Motion
gait_params = walk
lag = int(update_time/sim_dt)
gg = BoltHumanoidMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)

gg.update_gait_params(gait_params, sim_t)

plot_time = np.inf

try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()
viz.display(q0)
v_des = np.array([-0.0, 0.0, 0.0])
w_des = 0.0

xs_plan, us_plan, f_plan = gg.optimize(q0, v0, np.round(sim_t,3), v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
gg.plot(q, v, plot_force=True)

for ind in range(10000):
    viz.display(xs_plan[ind][:pin_robot.model.nq])
