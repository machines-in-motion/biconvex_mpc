## Demo for robot doing a cartwheel
## Author : Avadesh Meduri & Paarth Shah
## Date : 04/08/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen_bolt import BoltHumanoidAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.bolt_jump import plan

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data
urdf = BoltHumanoidConfig.urdf_path

viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
pin_robot = BoltHumanoidConfig.buildRobotWrapper()

q0 = np.array(BoltHumanoidConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_ANKLE", "FR_ANKLE", "L_WRIST", "R_WRIST"]
motion_time = plan.T
sim_t = 0.0
sim_dt = .001
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = BoltHumanoidAcyclicGen(pin_robot, urdf)
q, v = q0, v0
mg.update_motion_params(plan, q, sim_t)
mg.params.use_offline_traj = False

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

xs_plan, us_plan, f_plan = mg.optimize(q, v, np.round(sim_t,3))
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
mg.plot(q, v, plot_force=False)

for ind in range(int(motion_time/sim_dt)-1):
    if(ind%3==0):
        viz.display(xs_plan[ind][:pin_robot.model.nq])
