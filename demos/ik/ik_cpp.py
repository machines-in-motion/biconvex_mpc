## This is demo for the inverse kinematics C++ version
## Author : Avadesh Meduri 
## Date : 22/04/2021

import numpy as np
from inverse_kinematics_cpp import InverseKinematics

from robot_properties_solo.config import Solo12Config

robot = Solo12Config.buildRobotWrapper()
dt = 5e-2
T = 1.0

q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, np.zeros(robot.model.nv)])

print(robot.model.nq, robot.model.nv)

des_pos = np.tile(np.array([0,0,0]), (int(T/dt),1))

ik = InverseKinematics(Solo12Config.urdf_path, dt, T)
# ik.add_position_tracking_task(robot.model.getFrameId("FL_FOOT"), 0.0, T, des_pos, 1e2, "FL_pos_task")
# ik.setup_costs()

# ik.optimize(x0)
