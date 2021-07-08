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
trot.stance_percent = [0.75, 0.75, 0.75, 0.75]
trot.gait_dt = 0.05
trot.phase_offset = [0.0, 0.5, 0.5, 0.0]

# IK
trot.state_wt = np.array([0., 0, 1000] + [100] * 3 + [2.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [500] * 3 + [10.0] *(pin_robot.model.nv - 6))
trot.swing_wt = [1e2,5e3]
trot.cent_wt = [1e3, 1e1]
trot.step_ht = 0.08
trot.nom_ht = 0.22
trot.reg_wt = [5e-3, 7e-4]

# Dyn 
trot.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+2, 1e+2, 1e+3, 1e+4, 1e+4, 1e4])
trot.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+4, 1e+2, 1e+2, 1e+3, 1e+5, 1e+5, 1e+5])
trot.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
trot.nom_ht = 0.25
trot.rho = 5e+4
trot.ori_correction = [0.5, 0.5, 0.5]

# Gains
trot.kp = 3.0
trot.kd = 0.05

#### Walking #######################################
walk = BiconvexMotionParams("solo12", "walk")

# Cnt
walk.gait_period = 1.0
walk.stance_percent = [0.8, 0.8, 0.8, 0.8]
walk.gait_dt = 0.05
walk.phase_offset = [0.0, 0.6, 0.2, 0.8]

# IK
walk.state_wt = np.array([0., 0, 1000] + [100] * 3 + [2.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [500] * 3 + [10.0] *(pin_robot.model.nv - 6))
walk.swing_wt = [1e2,5e3]
walk.cent_wt = [1e3, 1e1]
walk.step_ht = 0.05
walk.nom_ht = 0.22
walk.reg_wt = [5e-3, 7e-4]

# Dyn 
walk.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+4, 1e+1, 1e+3, 5e+4, 1e+4, 1e+5])
walk.W_X_ter = np.array([1e-5, 1e-5, 1e+5, 1e+4, 1e+1, 1e+3, 5e+4, 1e+4, 1e+5])
walk.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
walk.nom_ht = 0.25
walk.rho = 5e+4
walk.ori_correction = [0.5, 0.5, 0.5]

# Gains
walk.kp = 3.0
walk.kd = 0.05