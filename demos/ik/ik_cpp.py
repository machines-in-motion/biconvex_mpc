## This is demo for the inverse kinematics C++ version
## Author : Avadesh Meduri 
## Date : 22/04/2021

import numpy as np
from inverse_kinematics_cpp import InverseKinematics

from robot_properties_solo.config import Solo12Config

# robot = Solo12Config.buildRobotWrapper()

dt = 5e-2
T = 1.0

ik = InverseKinematics(Solo12Config.urdf_path, dt, T)