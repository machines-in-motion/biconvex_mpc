## This class is the abstract class to store weights params for different gaits for robots
## Author : Avadesh Meduri & Paarth Shah
## Date : 7/7/21

import numpy as np

class BiconvexMotionParams:

    def __init__(self, robot_name, motion_name):

        self.robot_name = robot_name 
        self.motion_name = motion_name

        ## Contact Values
        self.gait_period = None     #Total Gait Period
        self.stance_percent = None  #Stance Percent
        self.gait_dt = 0.05         #Gait_dt
        self.phase_offset = None    #Phase Offsets
        self.step_ht = None         #Step Height
        self.contact_offsets = None #Contact offsets (Used to plan feet somewhere other than below the hip)

        ## Dynamic values
        self.W_X = None       #Running Cost State Weights
        self.W_X_ter = None   #Terminal Cost State Weights
        self.W_F = None       #Running Cost on Force Weights
        self.nom_ht = None    #Nominal Height
        self.rho = None       #ADMM Rho parameter

        self.ori_correction = None #Orientation correction

        ## Inverse Kinematic valuess

        self.swing_wt = np.zeros(2)
        self.cent_wt = np.zeros(2)

        self.state_wt = None #State Regularization
        self.ctrl_wt = None #Control Regularization

        ## Controller Gains
        self.kp = None
        self.kd = None