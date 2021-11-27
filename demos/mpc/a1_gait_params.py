## Contains a1 gait params
## Author : Paarth Shah
## Date : 7/7/21

import numpy as np
from weight_abstract import BiconvexMotionParams

#### Stand Still #########################################
still = BiconvexMotionParams("a1", "Stand")

# Cnt
still.gait_period = 0.5
still.stance_percent = [1.0, 1.0, 1.0, 1.0]
still.gait_dt = 0.05
still.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
still.state_wt = np.array([0., 0, 100] + [1000] * 3 + [10.0] * (12) \
                          + [0.00] * 3 + [100] * 3 + [5.0] *(12))

still.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [4.0] *(12)

still.swing_wt = [1e4, 1e4]
still.cent_wt = [0*5e+1, 5e+2]
still.step_ht = 0.13
still.nom_ht = 0.30
still.reg_wt = [5e-2, 1e-5]

# Dyn
still.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
still.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
still.W_F = np.array(4*[1e-1, 1e-1, 1e-1])
still.rho = 5e+4
still.ori_correction = [0.4, 0.5, 0.4]
still.gait_horizon = 1.5

# Gains
still.kp = 12.0
still.kd = 1.0


# #### 2-Leg Balance #########################################
# balance = BiconvexMotionParams("solo12", "Balance")
#
# # Cnt
# balance.gait_period = 0.15
# balance.stance_percent = [1.0, 0.0, 0.0, 1.0]
# balance.gait_dt = 0.05
# balance.phase_offset = [0.0, 0.0, 0.0, 0.0]
#
# # IK
# balance.state_wt = np.array([0., 0, 10] + [1000] * 3 + [1.0, 1.0, 1.0] + 3 * [0.0] + 3 * [0.0] + [1.0, 1.0, 1.0] \
#  \
#                             + [0.00] * 3 + [100] * 3 + [0.5] *(pin_robot.model.nv - 6))
#
# balance.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)
#
# balance.swing_wt = [2e3, 2e3]
# balance.cent_wt = [3e+5, 5e+1]
# balance.step_ht = 0.13
# balance.nom_ht = 0.26
# balance.reg_wt = [5e-2, 1e-5]
#
# # Dyn
# balance.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 2e+2, 1e+1, 1e+2, 1e+2, 1e2])
# balance.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 2e+2, 1e+1, 1e+2, 1e+2, 1e+2])
# balance.W_F = np.array(4*[1e-1, 1e-1, 1e-1])
# balance.rho = 5e+4
# balance.ori_correction = [0.4, 0.4, 0.4]
# balance.gait_horizon = 1.0
#
# # Gains
# balance.kp = 0.5
# balance.kd = 0.1
#
# #### Gallop Still #########################################
# gallop = BiconvexMotionParams("solo12", "Gallop")
#
# # Cnt
# gallop.gait_period = 0.5
# gallop.stance_percent = [0.35, 0.35, 0.35, 0.35] #FL, FR, HL, HR
# gallop.gait_dt = 0.05
# gallop.phase_offset = [0.0, 0.80, 0.70, 0.5]
#
# # IK
# gallop.state_wt = np.array([0.0, 0.0, 10.0] + [5000] * 3 + [0.0, 60.0, 60.0] * 4 \
#                            + [0.0, 0.0, 0.0]  + [1000] * 3 + [30.0, 30.0, 30.0] * 4)
#
# gallop.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)
#
# gallop.swing_wt = [1e4, 1e4]
# gallop.cent_wt = [5e+3, 5e+3]
# gallop.step_ht = 0.08
# gallop.nom_ht = 0.26
# gallop.reg_wt = [5e-2, 1e-5]
#
# # Dyn
# gallop.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e5])
# gallop.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
# gallop.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
# gallop.rho = 5e+4
# gallop.ori_correction = [0.6, 0.6, 0.4]
# gallop.gait_horizon = 2.0
#
# # Gains
# gallop.kp = 3.5
# gallop.kd = 0.1
#
#
#### Trot #########################################
trot = BiconvexMotionParams("a1", "Trot")

# Cnt
trot.gait_period = 0.5
trot.stance_percent = [0.6, 0.6, 0.6, 0.6]
trot.gait_dt = 0.05
trot.phase_offset = [0.0, 0.5, 0.5, 0.0]

# IK
trot.state_wt = np.array([0., 0, 100] + [1000, 1000, 1000] + [10.0] * (12) \
                         + [0.00] * 3 + [100, 100, 100] + [5.0] *(12))

trot.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [2.0] *(12)

trot.swing_wt = [1e4, 1e4]
trot.cent_wt = [5e+1, 5e+2]
trot.step_ht = 0.1
trot.nom_ht = 0.27
trot.reg_wt = [5e-2, 1e-5]

# Dyn
trot.W_X =        np.array([3e-3, 3e-3, 1e+5, 1e+2, 1e+2, 2e+2, 1e+4, 1e+5, 1e4])
trot.W_X_ter = 10*np.array([1e+5, 1e+5, 5e+5, 1e+2, 1e+2, 2e+2, 1e+5, 1e+5, 1e5])
trot.W_F = np.array(4*[1e-1, 1e-1, 1e-1])
trot.rho = 5e+4
trot.ori_correction = [0.30, 0.4, 0.4]
trot.gait_horizon = 1.25
trot.kp = 12.0
trot.kd = 2.0
#
# #### Trot with Turning #########################################
# trot_turn = BiconvexMotionParams("solo12", "Trot_turn")
#
# # Cnt
# trot_turn.gait_period = 0.5
# trot_turn.stance_percent = [0.6, 0.6, 0.6, 0.6]
# trot_turn.gait_dt = 0.05
# trot_turn.phase_offset = [0.0, 0.4, 0.4, 0.0]
#
# # IK
# trot_turn.state_wt = np.array([0., 0, 10] + [1000, 1000, 10] + [1.0] * (pin_robot.model.nv - 6) \
#                               + [0.00] * 3 + [100, 100, 10] + [0.5] *(pin_robot.model.nv - 6))
#
# trot_turn.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)
#
# trot_turn.swing_wt = [1e4, 1e4]
# trot_turn.cent_wt = [0*5e+1, 5e+2]
# trot_turn.step_ht = 0.05
# trot_turn.nom_ht = 0.2
# trot_turn.reg_wt = [5e-2, 1e-5]
#
# # Dyn
# trot_turn.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
# trot_turn.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
# trot_turn.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
# trot_turn.rho = 5e+4
# trot_turn.ori_correction = [0.0, 0.5, 0.4]
# trot_turn.gait_horizon = 1.0
# trot_turn.kp = 3.0
# trot_turn.kd = 0.05
#
#
# #### Walking #######################################
# walk = BiconvexMotionParams("solo12", "walk")
#
# # Cnt
# walk.gait_period = 0.6
# walk.stance_percent = [0.8, 0.8, 0.8, 0.8]
# walk.gait_dt = 0.05
# walk.phase_offset = [0.6, 0.0, 0.2, 0.8]
#
# # IK
# walk.state_wt = np.array([0., 0, 1000] + [1e3] * 3 + [0.5] * (pin_robot.model.nv - 6) \
#                          + [0.00] * 3 + [50] * 3 + [1e-2] *(pin_robot.model.nv - 6))
#
# walk.ctrl_wt = [1, 1, 10] + [10, 10, 20] + [5e-3] *(pin_robot.model.nv - 6)
#
# walk.swing_wt = [1e4,1e4]
# walk.cent_wt = [5e+1, 5e+2]
# walk.step_ht = 0.05
# walk.reg_wt = [5e-3, 7e-3]
#
# # Dyn
# walk.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 5e+3, 5e+3, 5e3])
# walk.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 1e+3, 1e+3, 1e+3])
# walk.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
# walk.nom_ht = 0.24
# walk.rho = 5e+4
# walk.ori_correction = [0.2, 0.4, 0.5]
# walk.gait_horizon = 0.5
#
# # Gains
# walk.kp = 3.5
# walk.kd = 0.15



# #### jump #########################################
# jump = BiconvexMotionParams("solo12", "Jump")
#
# # Cnt
# jump.gait_period = 0.5
# jump.stance_percent = [0.4, 0.4, 0.4, 0.4]
# jump.gait_dt = 0.05
# jump.phase_offset = [0.3, 0.3, 0.3, 0.3]
#
# # IK
# jump.state_wt = np.array([0., 0, 10] + [1000] * 3 + [1.0] * (pin_robot.model.nv - 6) \
#                          + [0.00] * 3 + [100] * 3 + [0.5] *(pin_robot.model.nv - 6))
#
# jump.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)
#
# jump.swing_wt = [1e4, 1e4]
# jump.cent_wt = [0*5e+1, 5e+2]
# jump.step_ht = 0.05
# jump.nom_ht = 0.25
# jump.reg_wt = [5e-2, 1e-5]
#
# # Dyn
# jump.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+4, 1e+4, 1e4])
# jump.W_X_ter = 10*np.array([1e+5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
# jump.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
# jump.rho = 5e+4
# jump.ori_correction = [0.2, 0.5, 0.4]
# jump.gait_horizon = 1.0
#
# # Gains
# jump.kp = 2.5
# jump.kd = 0.08