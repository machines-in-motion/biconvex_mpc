## Contains Talos gait params
## Author : Avadesh Meduri
## Date :

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_talos.config import TalosConfig

pin_robot = TalosConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data

eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
             "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]
n_eff = len(eff_names)

walk = BiconvexMotionParams("Talos", "walk")

# Cnt
walk.gait_period = 0.8
walk.stance_percent = 4*[0.6,] + 4*[0.6,]
walk.gait_dt = 0.1
walk.phase_offset = 4*[0.3,] + 4*[0.8,]

# IK
walk.state_wt = np.array([1e0, 1e4, 1e4] + [1e3] * 3 + \
                    [1e3,1e5,5e3,2e3,1e3,1e3] + \
                    [1e3,1e5,5e3,2e3,1e3,1e3] + \
                    2*[1e3,] +  \
                    4*[1e3,] + 3*[1e3,] + \
                    4*[1e3,] + 3*[1e3,] \
                         + [1e1] * 3 + [1e3] * 3 + [1e0] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [0, 0, 1] + [5e2, 5e2, 5e2] + [5e2] *(rmodel.nv - 6)

walk.swing_wt = [1e4, 1e3]
walk.cent_wt = [3*[1e+6,], 6*[1e-1,]]
walk.step_ht = 0.05
walk.nom_ht = 0.9
walk.reg_wt = [5e-2, 1e-5]


# Dyn
walk.W_X =     np.array([1e4, 1e4, 1e+5, 1e+3, 1e+2, 2e+2, 1e+4, 5e+4, 1e4])
walk.W_X_ter = 10.*np.array([1e+5, 1e5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
walk.W_F = np.array(8*[1e1, 1e1,5e1])
walk.rho = 1e4

walk.ori_correction = [0.2, 0.2, 0.2]
walk.gait_horizon = 1.5

# Gains
walk.kp = np.array([1e3,1e3,5e3,2e3,5e3,1e3] + # left leg
                    [1e3,1e3,5e3,2e3,5e3,1e3] + # right leg
                    2*[1e5,] +  # torso
                    4*[2e2,] + 3*[1e1,] + # left hand 
                    4*[2e2,] + 3*[1e1,]) # right hand

walk.kd = np.array(2*[5e1,5e1,5e1,2e1,1e1,1e1] + 
              2*[1.0e1,] + 
              4*[5e0,] + 3*[1e-1,] + 
              4*[5e0,] + 3*[1e0,])