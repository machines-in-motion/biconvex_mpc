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

jump = BiconvexMotionParams("Talos", "jump")

# Cnt
jump.gait_period = 1.0
jump.stance_percent = 4*[0.7,] + 4*[0.7,]
jump.gait_dt = 0.05
jump.phase_offset = 4*[0.0,] + 4*[0.0,]

# IK
jump.state_wt = np.array([1e0, 1e0, 1e0] + [1e3] * 3 + \
                    [1e3,1e5,5e3,2e2,1e3,1e3] + \
                    [1e3,1e5,5e3,2e2,1e3,1e3] + \
                    2*[1e5,] +  \
                    4*[1e5,] + 3*[1e5,] + \
                    4*[1e5,] + 3*[1e5,] \
                         + [1e0] * 3 + [1e2] * 3 + [1e2] *(pin_robot.model.nv - 6))

jump.ctrl_wt = [0, 0, 1] + [5e2, 5e2, 5e2] + [5e2] *(rmodel.nv - 6)

jump.swing_wt = [1e5, 1e4]
jump.cent_wt = [3*[1e+8,], 6*[1e-1,]]
jump.step_ht = 0.15
jump.nom_ht = 0.65
jump.reg_wt = [5e-2, 1e-5]


# Dyn
jump.W_X =     np.array([1e4, 1e4, 1e+2, 1e+3, 1e+2, 2e+2, 1e+4, 5e+4, 1e4])
jump.W_X_ter = 10.*np.array([1e+5, 1e5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
jump.W_F = np.array(8*[1e1, 1e1,5e1])
jump.rho = 1e4

jump.ori_correction = [0.2, 0.2, 0.2]
jump.gait_horizon = 2.2
# Gains
jump.kp = np.array([1e5,1e5,1e5,1e5,1.5e3,1e3] + # left leg
                    [1e5,1e5,1e5,1e5,1.5e3,1e3] + # right leg
                    2*[1e5,] +  # torso
                    4*[2e2,] + 3*[1e1,] + # left hand 
                    4*[2e2,] + 3*[1e1,]) # right hand

jump.kd = np.array(2*[5e1,5e1,5e1,3e1,3e1,1e1] + 
              2*[1.0e1,] + 
              4*[5e0,] + 3*[1e-1,] + 
              4*[5e0,] + 3*[1e0,])