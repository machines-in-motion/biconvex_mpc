## Contains solo 12 gait params
## Author : Avadesh Meduri
## Date : 7/7/21

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_solo.config import Solo12Config

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

#### jump #########################################
jump = BiconvexMotionParams("solo12", "Jump")

# Cnt
jump.gait_period = 0.5
jump.stance_percent = [0.4, 0.4, 0.4, 0.4]
jump.gait_dt = 0.05
jump.phase_offset = [0.3, 0.3, 0.3, 0.3]

# IK
jump.state_wt = np.array([0., 0, 10] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100] * 3 + [0.5] *(pin_robot.model.nv - 6))

jump.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

jump.swing_wt = [1e4, 1e4]
jump.cent_wt = [3*[0*5e+1,], 6*[5e+2,]]
jump.step_ht = 0.05
jump.nom_ht = 0.25
jump.reg_wt = [5e-2, 1e-5]

# Dyn
jump.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
jump.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
jump.W_F = np.array(4*[1e+1, 1e+1, 1.5e+1])
jump.rho = 5e+4
jump.ori_correction = [0.2, 0.5, 0.4]
jump.gait_horizon = 2.0

# Gains
jump.kp = 2.5
jump.kd = 0.08
