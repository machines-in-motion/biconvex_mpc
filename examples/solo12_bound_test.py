import time
import numpy as np
import pinocchio as pin
from matplotlib import pyplot as plt

from robot_properties_solo.config import Solo12Config
from mpc.abstract_cyclic_gen import SoloMpcGaitGen
from motions.weight_abstract import BiconvexMotionParams
# from solo12_gait_params import trot, w

robot = Solo12Config.buildRobotWrapper()
viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
pin_robot = Solo12Config.buildRobotWrapper()

bound = BiconvexMotionParams("solo12", "Bound")
#
# Cnt
bound.gait_period = 0.3
bound.stance_percent = [0.5, 0.5, 0.5, 0.5]
bound.gait_dt = 0.05
bound.phase_offset = [0.0, 0.0, 0.5, 0.5]
#
# IK
bound.state_wt = np.array([0., 0, 1e3] + [10, 10, 10] + [50.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100, 10, 100] + [0.5] *(pin_robot.model.nv - 6))
#
bound.ctrl_wt = [0.5, 0.5, 0.5] + [1, 1, 1] + [0.5] *(pin_robot.model.nv - 6)
#
bound.swing_wt = [1e4, 1e4]
bound.cent_wt = [5e+1, 5e+2]
bound.step_ht = 0.07
bound.reg_wt = [7e-3, 7e-5]
#
# Dyn
bound.W_X =        np.array([1e-5, 1e-5, 5e+4, 1e1, 1e1, 1e+3, 5e+3, 1e+4, 5e+3])
bound.W_X_ter = 10*np.array([1e-5, 1e-5, 5e+4, 1e1, 1e1, 1e+3, 1e+4, 1e+4, 1e+4])
bound.W_F = np.array(4*[1e1, 1e+1, 1.5e+1])
bound.nom_ht = 0.25
bound.rho = 5e+4
bound.ori_correction = [0.2, 0.8, 0.8]
bound.gait_horizon = 20.5
#
# Gains
bound.kp = 3.0
bound.kd = 0.05

## Motion
gait_params = bound

## robot config and init
urdf_path = Solo12Config.urdf_path
gait_time = gait_params.gait_period
dt = 5e-2

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
# q0[7:13] = 2 * [0.0, -0.8, 1.6]

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

w_des = 0.0
v_des = np.array([-0.4,0.0, 0])

update_time = 0.02 # sec (time of lag)

n = 1

sim_t = 0.0
step_t = 0
sim_dt = .001
plan_freq = 15

gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)
gg.update_gait_params(gait_params, sim_t)

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

xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)

for ind in range(int(plan_freq/sim_dt)):
    viz.display(xs_plan[ind][:robot.model.nq])
