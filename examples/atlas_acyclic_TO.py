## Demo for bolt humanoid doing a jumping motion
## Author : Majid Khadiv
## Date : 05/04/2022

import time
import numpy as np
import pinocchio as pin

from robot_properties_atlas.atlaswrapper import AtlasConfig, AtlasRobot
from mpc.abstract_acyclic_gen_atlas import AtlasAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.atlas_jump import plan

pin_robot = AtlasConfig.buildRobotWrapper()
rmodel = pin_robot.model
urdf = AtlasConfig.urdf_path

viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
pin_robot = AtlasConfig.buildRobotWrapper()

q0 = np.array(AtlasConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

plan_freq = 15
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = AtlasAcyclicGen(pin_robot, urdf)
# q, v = pin_robot.get_state()
q = q0
v = v0
mg.update_motion_params(plan, q, sim_t)

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

v_des = np.array([0.0, 0.0, 0.0])
w_des = 0.0
xs_plan, us_plan, f_plan = mg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
mg.plot(q, v, plot_force=True)

for ind in range(int(plan_freq/sim_dt)):
    if(ind%4==0):
        viz.display(xs_plan[ind][:pin_robot.model.nq])
