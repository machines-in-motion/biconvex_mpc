## Contains Atlas gait params
## Author : Avadesh Meduri & Paarth Shah
## Date : 7/7/21

import numpy as np
from weight_abstract import BiconvexMotionParams

#### Balance "Gait" #########################################
balance = BiconvexMotionParams("atlas", "Balance")

# Gait Information
balance.gait_period = 0.5
balance.stance_percent = [1.0, 1.0]
balance.gait_dt = 0.05
balance.phase_offset = [0.0, 0.0]
balance.swing_wt = [1e4, 1e4]
balance.step_ht = 0.13

# Inverse Kinematics
balance.state_wt = np.array([0., 0, 10] + [1000.0] * 3 +
                            [1.0] * (29) +
                            [0.00] * 3 + [100.0] * 3 +
                            [1.0] * (29) )


balance.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] * (29) # Penalty on control (aka accelerations)

# Centroidal
balance.cent_wt = [0*5e+1, 5e+2]
balance.nom_ht = 0.935

# Regularization Weight
balance.reg_wt = [5e-2, 1e-5]

# Dynamics
balance.W_X =        np.array([1e-5, 1e-5, 5e+7, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
balance.W_X_ter = 10*np.array([1e+5, 1e+5, 5e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
balance.W_F = np.array(4*[1e-1, 1e-1, 1e-1])
balance.rho = 5e+4
balance.ori_correction = [0.4, 0.5, 0.4]
balance.gait_horizon = 2.0

# Controller Gains
balance.kp = 50.0
balance.kd = 20.0
