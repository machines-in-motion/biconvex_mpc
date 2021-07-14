## Contains solo 12 gait params
## Author : Avadesh Meduri
## Date : 7/7/21

import numpy as np
from weight_abstract import BiconvexMotionParams
from robot_properties_solo.config import Solo12Config

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

#### Trot #########################################
trot = BiconvexMotionParams("solo12", "Trot")

# Cnt
trot.gait_period = 0.5
trot.stance_percent = [0.6, 0.6, 0.6, 0.6]
trot.gait_dt = 0.05
trot.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
trot.state_wt = np.array([0., 0, 10] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100] * 3 + [0.5] *(pin_robot.model.nv - 6))

trot.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

trot.swing_wt = [1e4, 1e4]
trot.cent_wt = [0*5e+1, 5e+2]
trot.step_ht = 0.05
trot.nom_ht = 0.2
trot.reg_wt = [5e-2, 1e-5]

# Dyn 
trot.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
trot.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
trot.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
trot.rho = 5e+4
trot.ori_correction = [0.0, 0.5, 0.4]
trot.gait_horizon = 1.0

# Gains
trot.kp = 3.0
trot.kd = 0.1

#### Walking #######################################
walk = BiconvexMotionParams("solo12", "walk")

# Cnt
walk.gait_period = 0.6
walk.stance_percent = [0.8, 0.8, 0.8, 0.8]
walk.gait_dt = 0.05
walk.phase_offset = [0.6, 0.0, 0.2, 0.8]

# IK
walk.state_wt = np.array([0., 0, 1000] + [1e3] * 3 + [0.5] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [50] * 3 + [1e-2] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [1, 1, 10] + [10, 10, 20] + [5e-3] *(pin_robot.model.nv - 6)

walk.swing_wt = [1e4,1e4]
walk.cent_wt = [5e+1, 5e+2]
walk.step_ht = 0.05
walk.reg_wt = [5e-3, 7e-3]

# Dyn 
walk.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 2e+2, 3e+4, 3e+4, 5e4])
walk.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 2e+2, 1e+3, 1e+3, 1e+3])
walk.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
walk.nom_ht = 0.2
walk.rho = 5e+4
walk.ori_correction = [0.2, 0.4, 0.6]
walk.gait_horizon = 0.5

# Gains
walk.kp = 3.5
walk.kd = 0.15
#### Bound #######################################

bound = BiconvexMotionParams("solo12", "bound")

# Cnt
bound.gait_period = 0.3
bound.stance_percent = [0.5, 0.5, 0.5, 0.5]
bound.gait_dt = 0.05
bound.phase_offset = [0.0, 0.0, 0.5, 0.5]

# IK
bound.state_wt = np.array([0., 0, 1e3] + [10, 10, 10] + [100.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [100, 10, 100] + [0.5] *(pin_robot.model.nv - 6))

bound.ctrl_wt = [0.5, 0.5, 0.5] + [1, 1, 1] + [0.5] *(pin_robot.model.nv - 6)

bound.swing_wt = [1e4, 1e4]
bound.cent_wt = [5e+1, 1e+2]
bound.step_ht = 0.05
bound.reg_wt = [7e-3, 7e-5]

# Dyn 
bound.W_X =        np.array([1e-5, 1e-5, 5e+4, 5, 5, 5e+1, 1e+4, 1e+2, 1e+4])
bound.W_X_ter = 10*np.array([1e+5, 1e-5, 5e+4, 5, 5, 5e+1, 1e+4, 1e+2, 1e+4])
bound.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
bound.nom_ht = 0.2
bound.rho = 5e+4
bound.ori_correction = [0.3, 0.0, 0.8]
bound.gait_horizon = 1.0

# Gains
bound.kp = 3.0
bound.kd = 0.05