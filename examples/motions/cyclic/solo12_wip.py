## Contains solo 12 gait params
## Author : Avadesh Meduri
## Date : 7/7/21

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_solo.config import Solo12Config

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

#### Stand Still #########################################
still = BiconvexMotionParams("solo12", "Stand")

# Cnt
still.gait_period = 0.5
still.stance_percent = [1.0, 1.0, 1.0, 1.0]
still.gait_dt = 0.05
still.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
still.state_wt = np.array([0., 0, 10] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
                         + [0.00] * 3 + [100] * 3 + [0.5] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

still.swing_wt = [1e4, 1e4]
still.cent_wt = [0*5e+1, 5e+2]
still.step_ht = 0.13
still.nom_ht = 0.26
still.reg_wt = [5e-2, 1e-5]

# Dyn
still.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
still.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
still.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
still.rho = 5e+4
still.ori_correction = [0.4, 0.5, 0.4]
still.gait_horizon = 2.0

# Gains
still.kp = 3.0
still.kd = 0.1

#### Gallop Still #########################################
gallop = BiconvexMotionParams("solo12", "Gallop")

# Cnt
gallop.gait_period = 0.5
gallop.stance_percent = [0.35, 0.35, 0.35, 0.35] #FL, FR, HL, HR
gallop.gait_dt = 0.05
gallop.phase_offset = [0.0, 0.80, 0.70, 0.5]

# IK
gallop.state_wt = np.array([0.0, 0.0, 10.0] + [5000] * 3 + [0.0, 60.0, 60.0] * 4 \
                         + [0.0, 0.0, 0.0]  + [1000] * 3 + [30.0, 30.0, 30.0] * 4)

gallop.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

gallop.swing_wt = [1e4, 1e4]
gallop.cent_wt = [5e+3, 5e+3]
gallop.step_ht = 0.08
gallop.nom_ht = 0.26
gallop.reg_wt = [5e-2, 1e-5]

# Dyn
gallop.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e5])
gallop.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
gallop.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
gallop.rho = 5e+4
gallop.ori_correction = [0.6, 0.6, 0.4]
gallop.gait_horizon = 2.0

# Gains
gallop.kp = 3.5
gallop.kd = 0.1

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
walk.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 5e+3, 5e+3, 5e3])
walk.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 1e+3, 1e+3, 1e+3])
walk.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
walk.nom_ht = 0.24
walk.rho = 5e+4
walk.ori_correction = [0.2, 0.4, 0.5]
walk.gait_horizon = 0.5

# Gains
walk.kp = 3.5
walk.kd = 0.15

