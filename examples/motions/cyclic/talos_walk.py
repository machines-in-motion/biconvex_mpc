## Contains Talos gait params
## Author : Avadesh Meduri
## Date :

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_talos.config import TalosConfig

pin_robot = TalosConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data

eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint"]
n_eff = len(eff_names)

walk = BiconvexMotionParams("Talos", "walk")

# Cnt
walk.gait_period = 1.
walk.stance_percent = 2*[0.6,] + 2*[0.6,]
walk.gait_dt = 0.1
walk.phase_offset = 2*[0.,] + 2*[0.5,]

# IK
walk.state_wt = np.array([0., 0., 0.] + [1e5] * 3 + [2e5] * (pin_robot.model.nv - 6) \
                         + [0.] * 3 + [1e3] * 3 + [1e1] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [0, 0, 1] + [5e2, 5e2, 5e2] + [5e2] *(rmodel.nv - 6)

walk.swing_wt = [5e5, 5e3]
walk.cent_wt = [5e+5, 1e-3]
walk.step_ht = 0.3
walk.nom_ht = 0.9
walk.reg_wt = [5e-2, 1e-5]


# Dyn
walk.W_X =     np.array([0., 0., 0., 1e+3, 1e+2, 2e+2, 1e+4, 5e+4, 1e4])
walk.W_X_ter = 10.*np.array([0., 0., 0., 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
walk.W_F = np.array(4*[1e1, 1e1, 5e1])
walk.rho = 1e4

walk.ori_correction = [0.5, 0.8, 0.2]
walk.gait_horizon = 4.

# Gains
# 12 * (6 dof leg) + 2* (hip joint ) + 2* [ 4 + 2 (arms) + 8 *(for fingers) ] + 2* (neck)
walk.kp = np.array(12*[5e3,] + 2*[1e4,] + 4*[1e3,] + 2*[1e2,] + 8*[1e0,] + 4*[1e3,] \
                    + 2*[1e2,] + 8*[1e0,] + 2*[1e3,] )
walk.kd = np.array(12*[1e1,] + 2*[1.5e1,] + 4*[5e0,] + 2*[1e-1,] + 8*[0.005,] + 4*[5e0,]\
                     + 2*[1e0,] + 8*[0.005,] + 2*[5e0,] )
