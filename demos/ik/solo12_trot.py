## This contains a pure IK based troting motion plan
## Author : Avadesh Meduri
## Date : 26/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator

dt = 1e-2
T = 2.0

robot = Solo12Config.buildRobotWrapper()
q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, pin.utils.zero(robot.model.nv)])

pin.forwardKinematics(robot.model, robot.data, q0, pin.utils.zero(robot.model.nv))
pin.updateFramePlacements(robot.model, robot.data)

fl_loc = robot.data.oMf[robot.model.getFrameId("FL_FOOT")].translation
fr_loc = robot.data.oMf[robot.model.getFrameId("FR_FOOT")].translation
hl_loc = robot.data.oMf[robot.model.getFrameId("HL_FOOT")].translation
hr_loc = robot.data.oMf[robot.model.getFrameId("HR_FOOT")].translation

gg = GaitGenerator(robot, T, dt)
gg.create_swing_foot_task(fl_loc, fl_loc, 0.0, 0.5, 0.1, "FL_FOOT", "FL_ftc1", 1e3)
# gg.create_swing_foot_task(hr_loc, hr_loc, 0.0, 0.5, 0.1, "HR_FOOT", "HR_ftc1", 1e3)
# gg.create_contact_task(fr_loc, 0.0, 0.5, "FR_FOOT", "FR_vtc1", 1e3)
# gg.create_contact_task(hl_loc, 0.0, 0.5, "HL_FOOT", "HL_vtc1", 1e3)

# xs = gg.optimize(x0)

# for i in range(len(xs)):
#     time.sleep(0.01)
#     viz.display(xs[i][:robot_model.nq])