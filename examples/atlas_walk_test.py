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

gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)

#### walk #########################################
walk = BiconvexMotionParams("atlas", "Walk")

# Cnt
walk.gait_period = 0.5
walk.stance_percent = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
walk.gait_dt = 0.05
walk.phase_offset = np.array(4*[0.] + 4*[0.5])

# IK
walk.state_wt = np.array([0., 0, 1] + [1e3, 1e3, 1e3] + [1.0] * (pin_robot.model.nv - 6) \
                         + [0.0, 0.0, 0.0] + [1e2, 1e2, 1e2] + [0.5] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [0, 0, 1] + [5e8, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)


walk.swing_wt = [1e4, 1e4]
walk.cent_wt = [1e4, 1e3]
walk.step_ht = 0.1
walk.nom_ht = 1.12
walk.reg_wt = [5e0, 1e-5]

# Dyn
walk.W_X = np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 2e+5, 1e4])
walk.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 2e+5, 1e+5])
walk.W_F = np.array(4*[1e+1, 1e+1, 1.5e+1])
walk.rho = 5e+4
walk.ori_correction = [0.2, 0.5, 0.4]
walk.gait_horizon = 2.0

#  Gains
walk.kp = 2.5
walk.kd = 0.08


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
for ind in range(np.size(xs_plan[:][0])):
    viz.display(xs_plan[ind][:robot.model.nq])
