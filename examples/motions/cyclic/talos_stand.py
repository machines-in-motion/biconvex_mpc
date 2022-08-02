## Contains Atlas gait params
## Author : Majid Khadiv & Avadesh Meduri
## Date : 4/7/2021

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_talos.config import TalosConfig

pin_robot = TalosConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data

eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
             "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]
n_eff = len(eff_names)

#### Stand Still #########################################
still = BiconvexMotionParams("Talos", "Stand")

# Cnt
still.gait_period = 0.4
still.stance_percent = n_eff*[1.,]
still.gait_dt = 0.05
still.phase_offset = int(n_eff)*[0.0,]

# IK
still.state_wt = np.array([1e4, 1e4, 1e4] + [1e5] * 3 + [1e5] * (pin_robot.model.nv - 6) \
                         + [1e2] * 3 + [1e3] * 3 + [5.] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)

still.swing_wt = [1e5, 2e5]
still.cent_wt = [3*[5e+0,], 6*[1e+1,]]
still.step_ht = 0.
still.nom_ht = 0.9
still.reg_wt = [5e-2, 1e-5]


# Dyn
still.W_X =     np.array([5e+1, 5e+1, 1e+5, 1e-2, 1e-2, 1e1, 1e+3, 1e+3, 1e+4])
still.W_X_ter = 10.*np.array([1e3, 1e3, 1e+5, 1e+2, 1e+2, 2e3, 1e+2, 1e+2, 1e+2])
still.W_F = np.array(8*[1e1, 1e1,1e1])
still.rho = 1e4

still.ori_correction = [0.5, 0.5, 0.8]
still.gait_horizon = 1.5

# Gains
still.kp = np.array([1e2,1e2,1e3,2e3,1e3,1e3] + # left leg
                    [1e2,1e2,1e3,2e3,1e3,1e3] + # right leg
                    2*[1e2,] +  # torso
                    4*[1e2,] + 3*[1e1,] + # left hand 
                    4*[1e2,] + 3*[1e1,]) # right hand

still.kd = np.array(2*[1e1,1e1,1e1,2e1,2e1,2e1] + 
              2*[1.0e1,] + 
              4*[5e0,] + 3*[1e-1,] + 
              4*[5e0,] + 3*[1e0,])