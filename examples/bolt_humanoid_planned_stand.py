# This file plays the mpc motion only in viz

import time
import numpy as np
import pinocchio as pin
from matplotlib import pyplot as plt

from robot_properties_bolt.config import BoltHumanoidConfig
from mpc.abstract_acyclic_gen import BoltHumanoidAcyclicGen
from motions.weight_abstract import ACyclicMotionParams

# from plan_cartwheel import plan
robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
urdf = BoltHumanoidConfig.urdf_path


q0 = np.array(BoltHumanoidConfig.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

plan = ACyclicMotionParams("bolt_humanoid", "stand")

T = 1.
dt = 5e-2
plan.n_col = 30
# dt = T/30
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[0.3, 0, T]]

plan.cnt_plan = [[[ 1.,      0.0,   0.1235,  0.03343, 0.,  T    ],
                  [ 1.,      0.0,  -0.1235,  0.03343, 0.,  T    ],
                  [ 0.,      0.0,   0.1235,  0.03343, 0.,  T    ],
                  [ 0.,      0.0,  -0.1235,  0.03343, 0.,  T    ]]]


#  dynamic optimization params
plan.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e-4, 1e-4, 2e2, 3e+4, 3e+4, 3e+4])
plan.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
plan.W_F = np.array(2*[1e+1, 1e+1, 1e+1] + 2*[0., 0., 0.])
plan.rho = 5e+4

plan.X_nom = [[0., 0, 0.4096, 0, 0, 0, 0, 0.00, 0.0, 0.0, T]]

plan.X_ter = [0., 0, 0.4096, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.25, 0.25, -0.25, 0.25, 0.3, 0.4, 0., T]]

# ik optimization params

plan.cent_wt = [1e1, 1e4]
plan.cnt_wt = 1e1

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e2, 1e2, 1e2 ] + [5.0, 5.0, 5.0] + 3*[1e2, 1e2, 1e2] + \
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + 3*[3.5, 3.5, 3.5]
                      )

plan.state_reg = [np.hstack((x_reg1, [0, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, T]))]
plan.state_scale = [[1e-2, 0, T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[1e-4, 0, T]]

# controller details
plan.kp = [[2.5, 0, T]]
plan.kd = [[0.5, 0, T]]

try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
plan_freq = 0.1 # sec
q = q0
v = v0

lag = int(update_time/sim_dt)

mg = BoltHumanoidAcyclicGen(pin_robot, urdf)
mg.update_motion_params(plan, q, sim_t)

for o in range(1):
    print(q)
    print(v)
    xs, us, f = mg.optimize(q, v, sim_t)
    xs = xs[lag:]
    us = us[lag:]
    f = f[lag:]

    time.sleep(0.0005)
    for ind in range(int(plan_freq/sim_dt)):
        viz.display(q0)

    q = xs[int(plan_freq/sim_dt)-1][0:pin_robot.model.nq]
    v = xs[int(plan_freq/sim_dt)-1][pin_robot.model.nq:]

    sim_t += plan_freq

mg.plot(q, v, plot_force=False)
