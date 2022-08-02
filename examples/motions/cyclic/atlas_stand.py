## Contains Atlas gait params
## Author : Majid Khadiv & Avadesh Meduri
## Date : 4/7/2021

import numpy as np
from motions.weight_abstract import BiconvexMotionParams
from robot_properties_atlas.config import AtlasConfig

pin_robot = AtlasConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data

eff_names = ["l_foot_lt", "l_foot_rt", "l_foot_lb", "l_foot_rb", "r_foot_lt", "r_foot_rt", "r_foot_lb", "r_foot_rb"]
hip_names = ["l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz"]
n_eff = len(eff_names)

#### Stand Still #########################################
still = BiconvexMotionParams("atlas", "Stand")

# Cnt
still.gait_period = 0.15
still.stance_percent = n_eff*[1.,]
still.gait_dt = 0.02
still.phase_offset = int(n_eff)*[0.0,]

# IK
still.state_wt = np.array([1e4, 1e4, 1e4] + [1e5] * 3 + [1e5] * (pin_robot.model.nv - 6) \
                         + [1e2] * 3 + [1e3] * 3 + [5.] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)

still.swing_wt = [1e5, 2e5]
still.cent_wt = [3*[5e+2,], 6*[0*5e+1,]]
still.step_ht = 0.
still.nom_ht = 1.12
still.reg_wt = [5e-2, 1e-5]


# Dyn
still.W_X =     np.array([5e+3, 5e+3, 1e+5, 1e-2, 1e-2, 1e1, 1e-2, 1e-2, 1e-2])
still.W_X_ter = 10.*np.array([1e3, 1e3, 1e+5, 1e+2, 1e+2, 2e3, 1e+2, 1e+2, 1e+2])
still.W_F = np.array(8*[1e1, 1e1,5e0])
still.rho = 1e4

still.ori_correction = [0.5, 0.5, 0.5]
still.gait_horizon = 1

# Gains
still.kp = np.array([1e5, 1e5, 1e5, 1e2] + [1e2,1e2,1e2,1e2,1e1,1e1,1e1,0,0,1e3,1e3,0,0] \
               + [0,0,0,0,0,0,0,0,0,0,0,0,0])
still.kd = np.array([1e2, 1e2, 1e2, 1e1] + [1e1,1e1,1e1,1e1,1,1,1,0,0,1e1,1e1,0,0] \
                + [0, 0,0,0, 0,  0, 0,  0,0,0,0,0,0])