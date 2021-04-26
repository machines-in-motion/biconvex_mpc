## This is demo for the inverse kinematics C++ version
## Author : Avadesh Meduri 
## Date : 22/04/2021

import numpy as np
from inverse_kinematics_cpp import InverseKinematics

from robot_properties_solo.config import Solo12Config

robot = Solo12Config.buildRobotWrapper()
dt = 5e-2
T = 1.0

des_pos = np.array([0,0,0])

ik = InverseKinematics(Solo12Config.urdf_path, dt, T)
# ik.setup_costs()
# ik.add_position_tracking_task(robot.model.getFrameId("FL_FOOT"), 0.0, 0.5, des_pos, 1e2, "FL_pos_task")
