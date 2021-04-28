## This is demo for the inverse kinematics C++ version
## Author : Avadesh Meduri 
## Date : 22/04/2021

import time
import numpy as np
from inverse_kinematics_cpp import InverseKinematics


from robot_properties_solo.config import Solo12Config

robot = Solo12Config.buildRobotWrapper()
dt = 5e-2
T = 0.2

q0 = np.array(Solo12Config.initial_configuration)
x0 = np.concatenate([q0, np.zeros(robot.model.nv)])

stateWeights = np.array([0.] * 3 + [500.] * 3 + [0.01] * (robot.model.nv - 6) \
                    + [10.] * 6 + [5.0] *(robot.model.nv - 6))

# print(robot.model.nq, robot.model.nv)

des_pos_fl = np.tile(np.array([0.3946,   0.14695,  0]), (int(T/dt),1))
des_pos_fr = np.tile(np.array([0.3946,   -0.14695,  0]), (int(T/dt),1))
des_pos_hl = np.tile(np.array([0.0054,   0.14695,  0]), (int(T/dt),1))
des_pos_hr = np.tile(np.array([0.0054,   -0.14695,  0]), (int(T/dt),1))

des_vel_fl = np.tile(np.array([0.,   0,  0]), (int(T/dt),1))
des_vel_fr = np.tile(np.array([0.,   -0,  0]), (int(T/dt),1))
des_vel_hl = np.tile(np.array([0.,   0,  0]), (int(T/dt),1))
des_vel_hr = np.tile(np.array([0.,   -0,  0]), (int(T/dt),1))

des_com_pos = np.tile(np.array([0.,   0,  0.2]), (int(T/dt),1))
des_com_pos[:,0] = 0.1*np.linspace(0, len(des_com_pos), len(des_com_pos))

des_mom = np.tile(np.array([0.,   0,  0.0, 0, 0, 0]), (int(T/dt),1))
des_mom[:,0] = 0.5


ik = InverseKinematics(Solo12Config.urdf_path, dt, T)
ik.add_position_tracking_task(robot.model.getFrameId("FL_FOOT"), 0.0, T, des_pos_fl, 1e2, "FL_pos_task")
ik.add_position_tracking_task(robot.model.getFrameId("FR_FOOT"), 0.0, T, des_pos_fr, 1e2, "FR_pos_task")
ik.add_position_tracking_task(robot.model.getFrameId("HL_FOOT"), 0.0, T, des_pos_hl, 1e2, "HL_pos_task")
ik.add_position_tracking_task(robot.model.getFrameId("HR_FOOT"), 0.0, T, des_pos_hr, 1e2, "HR_pos_task")

# ik.add_velocity_tracking_task(robot.model.getFrameId("FL_FOOT"), 0.0, T, des_vel_fl, 1e1, "FL_vel_task")
# ik.add_velocity_tracking_task(robot.model.getFrameId("FR_FOOT"), 0.0, T, des_vel_fr, 1e1, "FR_vel_task")
# ik.add_velocity_tracking_task(robot.model.getFrameId("HL_FOOT"), 0.0, T, des_vel_hl, 1e1, "HL_vel_task")
# ik.add_velocity_tracking_task(robot.model.getFrameId("HR_FOOT"), 0.0, T, des_vel_hr, 1e1, "HR_vel_task")

# ik.add_com_position_tracking_task(0.0, T, des_com_pos, 1e3, "com_track", False)
ik.add_centroidal_momentum_tracking_task(0.0, T, des_mom, 1e3, "mom_track", False)

ik.add_state_regularization_cost(0, T, 5e-3, "state_reg", stateWeights, x0)
ik.add_ctrl_regularization_cost(0, T, 5e-4, "ctrl_reg")


ik.setup_costs()

st = time.time()
ik.optimize(x0)
et = time.time()
print("IK time : ", et - st)
xs = np.array (ik.get_xs())

np.savez("../motion_planner/dat_file/ik", xs = xs)

print(xs.shape)