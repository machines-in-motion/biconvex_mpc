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

update_time = 0.0 # sec (time of lag)

# gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)

#### Stand Still #########################################
still = BiconvexMotionParams("atlas", "Stand")

# Cnt
still.gait_period = 0.5
still.stance_percent = n_eff*[1.,]
still.gait_dt = 0.05
still.phase_offset = int(n_eff)*[0.0,]

# IK
still.state_wt = np.array([1e2, 1e2, 1e2] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
                         + [0.00] * 3 + [100] * 3 + [.1] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)

still.swing_wt = [1e5, 1e5]
still.cent_wt = [5e+5, 5e+4]
still.step_ht = 0.
still.nom_ht = 1.1275
still.reg_wt = [5e-2, 1e-5]

# Dyn
still.W_X =     np.array([1e0, 1e0, 1e+5, 1e-4, 1e-4, 2e0, 3e+0, 1e+0, 3e+0])
still.W_X_ter = 10.*np.array([1e-3, 1e-5, 1e+3, 1e-1, 1e-1, 2e2, 1e+1, 1e+1, 1e+1])
still.W_F = np.array(8*[.0, 0., .0])
still.rho = 5e4

still.ori_correction = [0.0, 0.0, 0.0]
still.gait_horizon = 1

# Gains
still.kp = 3.0
still.kd = 0.1

sim_t = 0.0
sim_dt = 0.05
index = 0
pln_ctr = 0
q = q0
v = v0
plan_freq = still.gait_horizon * still.gait_period # sec


gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)
gg.update_gait_params(still, 0)

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

q = q0
v = v0
step_t = 0
n = 0

xs_plan, us_plan, f_plan = gg.optimize(q, v, 0., v_des, w_des)
q = xs_plan[0][0:pin_robot.model.nq]
v = xs_plan[0][pin_robot.model.nq:]
gg.plot(q, v, plot_force=True)

for ind in range(int(1000*(plan_freq/2-still.gait_dt))):
    viz.display(xs_plan[ind][:robot.model.nq])
