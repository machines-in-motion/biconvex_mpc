## Contains solo 12 gait params
## Author : Avadesh Meduri
## Date : 7/7/21

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_solo.config import Solo12Config

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

#### Trot #########################################
trot = BiconvexMotionParams("solo12", "Trot")

# Cnt
trot.gait_period = 0.5
trot.stance_percent = [0.6, 0.6, 0.6, 0.6]
trot.gait_dt = 0.05
trot.phase_offset = [0.0, 0.5, 0.5, 0.0]

# IK
trot.state_wt = np.array([0., 0, 10] + [1000, 1000, 1000] + [1.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100, 100, 100] + [0.5] *(pin_robot.model.nv - 6))

trot.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

trot.swing_wt = [3*[1e4,], 3*[1e4,]]
trot.cent_wt = [3*[0*5e+1,], 6*[5e+2,]]
trot.step_ht = 0.05
trot.nom_ht = 0.2
trot.reg_wt = [5e-2, 1e-5]

# Dyn 
trot.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
trot.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
trot.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
trot.rho = 5e+4
trot.ori_correction = [0.3, 0.5, 0.4]
trot.gait_horizon = 1.0
trot.kp = 3.0
trot.kd = 0.05

#### Trot with Turning #########################################
trot_turn = BiconvexMotionParams("solo12", "Trot_turn")

# Cnt
trot_turn.gait_period = 0.5
trot_turn.stance_percent = [0.6, 0.6, 0.6, 0.6]
trot_turn.gait_dt = 0.05
trot_turn.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
trot_turn.state_wt = np.array([0., 0, 10] + [1000, 1000, 10] + [1.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100, 100, 10] + [0.5] *(pin_robot.model.nv - 6))

trot_turn.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

trot_turn.swing_wt = [3*[1e4,], 3*[1e4,]]
trot_turn.cent_wt = [0*5e+1, 5e+2]
trot_turn.step_ht = 0.05
trot_turn.nom_ht = 0.2
trot_turn.reg_wt = [5e-2, 1e-5]

# Dyn 
trot_turn.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e5])
trot_turn.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
trot_turn.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
trot_turn.rho = 5e+4
trot_turn.ori_correction = [0.0, 0.5, 0.4]
trot_turn.gait_horizon = 1.0
trot_turn.kp = 3.0
trot_turn.kd = 0.05