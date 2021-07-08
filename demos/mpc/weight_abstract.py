## This class is the abstract class to store weights params for different gaits for robots
## Author : Avadesh Meduri
## Date : 7/7/21

import numpy as np

class BiconvexMotionParams:

    def __init__(self, robot_name, motion_name):

        self.robot_name = robot_name 
        self.motion_name = motion_name

        ## Cnt Vals
        self.gait_period = None
        self.stance_percent = None
        self.gait_dt = 0.05
        self.phase_offset = None

        self.step_ht = None

        ## Dyn vals
        self.W_X = None        
        self.W_X_ter = None
        self.W_F = None
        self.nom_ht = None
        self.rho = None

        self.ori_correction = None

        ## IK vals

        self.swing_wt = np.zeros(2)
        self.cent_wt = np.zeros(2)

        self.state_wt = None

        ## Gains
        self.kp = None
        self.kd = None