## Contains Bolt humanoid gait params
## Author : Majid Khadiv
## Date : ..

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_bolt.config import BoltHumanoidConfig

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
urdf_path = BoltHumanoidConfig.urdf_path

#### walk #########################################
walk = BiconvexMotionParams("bolt_humanoid", "Walk")

# Cnt
walk.gait_period = 0.5
walk.stance_percent = [0.6, 0.6, 0.0, 0.0]
walk.gait_dt = 0.05
walk.phase_offset = [0., 0.5, 0.0, 0.0]

# IK
walk.state_wt = np.array([0., 0, 1] + [1e3, 1e3, 1e3] +  2 * [1., 1., 1.] + [1e3, 10., 10.] \
                        + [0.0, 0.0, 0.0] + [1e2, 1e2, 1e2] +  2*[0.01, 0.01, 0.01] + [3.5, 0.01, 0.01])

walk.ctrl_wt = [0, 0, 1] + [5e8, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

walk.swing_wt = [1e4, 1e4]
walk.cent_wt = [0., 0.]
walk.step_ht = 0.1
walk.nom_ht = 0.4
walk.reg_wt = [5e0, 1e-5]

# Dyn
walk.W_X = np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 2e+5, 1e4])
walk.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 2e+5, 1e+5])
walk.W_F = np.array(4*[1e+1, 1e+1, 1.5e+1])
walk.rho = 5e+4
walk.ori_correction = [0.2, 0.5, 0.4]
walk.gait_horizon = 10

# Gains
walk.kp = 2.5
walk.kd = 0.08
