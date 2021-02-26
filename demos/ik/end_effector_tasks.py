## This file contains apis to create different end effector tasks
## for the inverse kinematics problem
## Author : Avadesh Meduri
## Date : 24/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

class EndEffectorTasks:

    def __init__(self):
        ## This class contains different tasks for swing foot trajectory
        ## This is inherited by the inversekinematics class
        ## Note :
            ## Paramteres such as self.dt etc.. are defined in the inverse kinematics
            ## class
        pass

    def add_trajectory_tracking_task(self, fid, st, et, traj, wt, cost_name):
        """
        This function creates a end effector tracking task in the ddp
        Input:
            fid : the frame id of the frame that should track the trajectory
            st : start time (sec)
            et : end time (sec)
            traj : the trajectory to be tracked (must have shape ((en - sn)/dt, 3))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            Mref = crocoddyl.FrameTranslation(fid, traj[i - sn])
            goalTrackingCost = crocoddyl.CostModelFrameTranslation(self.state, Mref)
            self.rcost_model_arr[i].addCost(cost_name, goalTrackingCost, wt)
            
    def add_velocity_tracking_cost(self, fid, st, et, traj, wt, cost_name):
        """
        This function creates a task for the end effector to track a desired velocity
        Input:
            fid : the frame id of the frame that should track the trajectory
            st : start time (sec)
            et : end time (sec)
            traj : the velocity trajectory to be tracked (must have shape ((en - sn)/dt, 6))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            Vref = crocoddyl.FrameMotion(fid, pin.Motion(traj[i - sn]))
            velTrackingCost = crocoddyl.CostModelFrameVelocity(self.state, Vref)
            self.rcost_model_arr[i].addCost(cost_name, velTrackingCost, wt)
