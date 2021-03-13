## This file contains the a gait generator (trotting, bounding, pacing)
## Author : Avadesh Meduri
## Date : 26/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics


class GaitGenerator:

    def __init__(self, robot, T, dt):
        """
        Input:
            robot : pinocchio robot 
            x0 : initial joint position and velocity configuration
            eff_names : foot names arr
            T : horizon of plan
            dt : discrertization
        """

        self.rmodel = robot.model
        self.rdata = robot.data
        self.ik = InverseKinematics(self.rmodel, dt, T)

        self.t = 0
        self.T = T
        self.dt = dt
        
    def create_swing_foot_task(self, x0, xT, st, et, sh, fname, cname, wt):
        """
        This creates a swing foot cost where the foot trajectory is a simple line based
        interpolation
        Input:
            x0 : location of foot at the start of trajectory
            xT : location of the foot at the end of trajecory
            st : start time of step in seconds
            et : end time of step in second
            sh : step height (assumed to reach this height midway)
            fname : frame name of foot
            cname : cost name
            wt : weight of cost
        """
        xMid = 0.5*(xT + x0)
        xMid[2] = sh

        self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname), \
                            st, st, x0, wt, cname)
        
        self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname), \
                            0.5*(st + et), 0.5*(st + et), xMid, 1e-1*wt, cname)
        
        self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname), \
        et, et, xT, wt, cname)
        
    def create_contact_task(self, x0, st, et, fname, cname, wt):
        """
        This creates a stationary trajectory for the foot that is on the ground
        Input:
            x0 : location of foot at the start of trajectory
            st : start time of step in seconds
            et : end time of step in second
            fname : frame name of foot
            cname : cost name
            wt : weight of cost
        """

        N = int(np.round(((et - st)/self.dt),2))
        pos_traj = np.tile(x0, (N,1))
        self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname), \
                            st, et, pos_traj, wt, cname + "_pos")

    def create_centroidal_task(self, traj, st, et, cname, wt):
        """
        This creates a centroidal tracking task
        Input:
            traj : cmom traj
            st : start time of step in seconds
            et : end time of step in second
            cname : cost name
            wt : weight of cost
        """
        self.ik.add_centroidal_momentum_tracking_task(st, et, traj, wt, cname)

    def optimize(self, x0, wt_xreg = 1e-4, wt_ureg = 1e-5, state_wt = None):
        """
        This function optimizes the Ik motion
        Input:
            x0 : initial configuration (q , v)
            wt_reg : regularization of the weights
            wt_ureg : control regularization
            state_wt : regularization of the states variables 
                        (look at add state regularization cost for more details)
        """
        self.ik.add_state_regularization_cost(0, self.T, wt_xreg, "xReg", state_wt)
        self.ik.add_ctrl_regularization_cost(0, self.T, wt_ureg, "uReg")

        # setting up terminal cost model
        xRegCost = crocoddyl.CostModelState(self.ik.state)
        uRegCost = crocoddyl.CostModelControl(self.ik.state)


        self.ik.terminalCostModel.addCost("stateReg", xRegCost, wt_xreg)
        self.ik.terminalCostModel.addCost("ctrlReg", uRegCost, wt_ureg) 

        self.ik.setup_costs()

        xs = self.ik.optimize(x0) 

        return xs