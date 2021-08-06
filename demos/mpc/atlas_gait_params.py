import numpy as np
from weight_abstract import BiconvexMotionParams
from robot_properties_atlas.config import AtlasConfig

pin_robot = AtlasConfig.buildRobotWrapper()
urdf_path = AtlasConfig.urdf_path

#### Stand Still #########################################
still = BiconvexMotionParams("atlas", "Stand")

# Cnt
still.gait_period = 0.5
still.stance_percent = [1.0, 1.0, 1.0, 1.0]
still.gait_dt = 0.05
still.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
still.state_wt = np.array([0.] * 3 + [50.] * 3 + [0.01] * (pin_robot.model.nv - 6) \
                        + [10.] * 6 + [1.0] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

still.swing_wt = [0*1e4, 1e4]
still.cent_wt = [0*5e+1, 0*5e+2]
still.step_ht = 0.13
still.nom_ht = 0.95
still.reg_wt = [5e-2, 1e-5]

# Dyn
still.W_X =        np.array([1e+5, 1e+5, 1e+5, 1e+2, 1e+2, 2e+4, 1e+4, 1e+4, 1e4])
still.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+2, 1e+2, 2e+4, 1e+5, 1e+5, 1e+5])
still.W_F = np.array(2*[1e-3, 1e-3, 1e-3])
still.rho = 5e+4
still.ori_correction = [0.0, 0.0, 0.0]
still.gait_horizon = 2.0

# Gains
still.kp = 3.0
still.kd = 0.1
