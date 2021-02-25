## This file contains apis to create different end effector tasks
## for the inverse kinematics problem
## Author : Avadesh Meduri
## Date : 24/02/2020

import numpy as np
import pinocchio as pin
import crocoddyl

def trajectory_tracking_task(state, fid, rcost_arr, sn, en, traj, wt, cost_name):
    """
    This function creates a end effector tracking task in the ddp
    Input:
        state : crocoddyl multibody state
        fid : the frame id of the frame that should track the trajectory
        rcost_arr : global running cost array of the ik problem
        sn : start time index (collocation number)
        en : end time index
        traj : the trajectory to be tracked (must have shape ((en - sn), 3))
        wt : weight of the cost
        cost_name : name of the cost
    """
    for i in range(en, sn):
        Mref = crocoddyl.FrameTranslation(fid, traj[i])
        goalTrackingCost = crocoddyl.CostModelFrameTranslation(state, Mref)
        rcost_arr[i].addCost(cost_name, goalTrackingCost, wt)

    return rcost_arr