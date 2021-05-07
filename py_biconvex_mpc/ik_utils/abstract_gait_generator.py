## This file contains the a gait generator (trotting, bounding, pacing)
## Author : Avadesh Meduri
## Date : 26/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

# from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
from inverse_kinematics_cpp import InverseKinematics


class AbstractGaitGenerator:

    def __init__(self, pin_model, pin_data, r_urdf, T, dt):
        """
        Input:
            robot : robot urdf
            r_urdf : robot urdf
            x0 : initial joint position and velocity configuration
            eff_names : foot names arr
            T : horizon of plan
            dt : discrertization
        """

        self.rmodel = pin_model
        self.rdata = pin_data
        self.ik = InverseKinematics(r_urdf, dt, T)
        # self.ik = InverseKinematics(self.rmodel, dt, T)

        self.t = 0
        self.T = T
        self.dt = dt

    def create_swing_foot_task(self, x0, xT, st, et, sh, fname, cname, wt, isTerminal = False):
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
        if not isTerminal:
            self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname),et, et, xT, wt, cname)

    def create_contact_task(self, x0, st, et, fname, cname, wt, isTerminal = False):
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
        if not isTerminal:
            N = int(np.round(((et - st)/self.dt),2))
            pos_traj = np.tile(x0, (N,1))
            self.ik.add_position_tracking_task(self.rmodel.getFrameId(fname), \
                                               st, et, pos_traj, wt, cname + "_pos")

        else:
            self.ik.add_terminal_position_tracking_task(self.rmodel.getFrameId(fname), \
                                                        x0, wt, cname + "term_pos")


    def create_centroidal_task(self, traj, st, et, cname, wt, isTerminal = False):
        """
        This creates a centroidal tracking task
        Input:
            traj : cmom traj
            st : start time of step in seconds
            et : end time of step in second
            cname : cost name
            wt : weight of cost
            isTerminal : True if this is to be added in the terminal cost
        """
        self.ik.add_centroidal_momentum_tracking_task(st, et, traj, wt, cname, isTerminal)

    def optimize(self, x0, state_wt, x_reg, wt_xreg = 1e-4, wt_ureg = 1e-5):
        """
        This function optimizes the Ik motion
        Input:
            x0 : initial configuration (q , v)
            wt_reg : regularization of the weights
            wt_ureg : control regularization
            state_wt : regularization of the states variables
                        (look at add state regularization cost for more details)
        """
        self.ik.add_state_regularization_cost(0, self.T, wt_xreg, "xReg", state_wt, x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, self.T, wt_ureg, "uReg", False)

        self.ik.add_state_regularization_cost(0, self.T, wt_xreg, "xReg", state_wt, x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, self.T, wt_ureg, "uReg", True)

        self.ik.setup_costs()

        self.ik.optimize(x0)

        self.xs = np.array (self.ik.get_xs())
        self.us = None

        return self.xs, self.us

    def compute_optimal_com_and_mom(self):
        """
        This function computes the optimal momentum based on the solution
        """

        opt_mom = np.zeros((len(self.xs), 6))
        opt_com = np.zeros((len(self.xs), 3))
        m = pin.computeTotalMass(self.rmodel)
        for i in range(len(self.xs)):
            q = self.xs[i][:self.rmodel.nq]
            v = self.xs[i][self.rmodel.nq:]
            pin.forwardKinematics(self.rmodel, self.rdata, q, v)
            pin.computeCentroidalMomentum(self.rmodel, self.rdata)
            opt_com[i] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
            opt_mom[i] = np.array(self.rdata.hg)
            opt_mom[i][0:3] /= m

        return opt_com, opt_mom