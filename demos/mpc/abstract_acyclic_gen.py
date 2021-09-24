## This file creates the costs for the kino-dyn planner
## Author : Avadesh Meduri
## Date : 23/09/2021

import time
import numpy as np
from numpy.lib.arraysetops import isin
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from biconvex_mpc_cpp import KinoDynMP
import math

class SoloAcyclicGen:

    def __init__(self, robot, r_urdf, freq):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            freq : planning frequency
        """
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        self.freq = freq
        # --- Set up Dynamics ---
        self.m = pin.computeTotalMass(self.rmodel)

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.hip_names = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]
        self.n_eff = 4

        # Set up constraints for Dynamics
        self.bx = 0.45
        self.by = 0.45
        self.bz = 0.45
        self.fx_max = 15.0
        self.fy_max = 15.0
        self.fz_max = 15.0

    def update_motion_params(self, weight_abstract, t):
        """
        Updates the gaits
        Input:
            weight_abstract : the parameters of the gaits
            t : time
        """
        self.params = weight_abstract
        self.horizon = self.params.n_col
        self.ik_horizon = self.params.n_col
        self.kd = KinoDynMP(self.r_urdf, self.m, len(self.eff_names), self.horizon, self.ik_horizon)
        self.kd.set_com_tracking_weight(self.params.cent_wt[0])
        self.kd.set_mom_tracking_weight(self.params.cent_wt[1])
        self.ik = self.kd.return_ik()
        self.mp = self.kd.return_dyn()

        self.mp.set_rho(self.params.rho)

        # For interpolation (should be moved to the controller)
        self.size = min(self.ik_horizon, int(self.freq/self.params.dt_arr[0]) + 2)
        # don't know if this is a good / robust way for interpolation (need a way to do this properly)
        if self.freq > self.params.dt_arr[0]:
            self.size -= 1
        self.xs_int = np.zeros((self.rmodel.nq + self.rmodel.nv, self.size))
        self.us_int = np.zeros((self.rmodel.nv, self.size))
        self.f_int = np.zeros((4*len(self.eff_names), self.size))

    def create_contact_plan(self, q, v, t, make_cyclic = False):
        """
        Creates contact plan based on moving horizon
        Input:
            q : current joint configuration
            v : current joint velocity
            t : current time into the plan
        """
        self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
        ft = t
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            for k in range(len(self.params.cnt_plan)):
                ## making an assumption that each phase is fixed for each leg
                ## can be changed
                if ft <= self.params.cnt_plan[-1][0][5]:
                    if ft >= self.params.cnt_plan[k][0][4] and ft < self.params.cnt_plan[k][0][5]:
                        i += 1
                        for j in range(len(self.eff_names)):
                            ## not sure how to update the plan based on current foot step locations
                            ## should discuss this
                            self.cnt_plan[i] = self.params.cnt_plan[k][j][0:4]

                else:
                    ## case where the t is larger than the plan horizon
                    if not make_cyclic:
                        self.cnt_plan[i] = self.params.cnt_plan[-1][j][0:4]
                        i += 1
                    else:
                        # make this cyclic later
                        pass
            
            if i == 0:
                dt = self.params.dt_arr[i] - np.round(np.remainder(t,self.params.dt_arr[i]),2)
                if dt == 0:
                    dt = self.params.dt_arr[i]
            else:
                dt = self.params.dt_arr[i]

            self.mp.set_contact_plan(self.cnt_plan[i], dt)


    def create_costs(self, q, v, t, make_cyclic = False):
        """
        Creates cost for the plan optimization
        Input:
            q : current joint configuration
            v : current joint velocity
            t : current time into the plan
        """
        # initial and terminal state
        
        self.x0 = np.hstack((q,v))
        
        X_init = np.zeros(9)
        pin.computeCentroidalMomentum(self.rmodel, self.rdata)
        X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q.copy(), v.copy())
        X_init[3:] = np.array(self.rdata.hg)
        X_init[3:6] /= self.m

        ## Dyn Costs ##
        X_nom = np.zeros((9*self.horizon))
        ft = t
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            if ft <= self.params.X_nom[-1][-1]:
                for k in range(len(self.params.X_nom)):
                    if ft > self.params.X_nom[k][9] and ft <= self.params.X_nom[k][10]:
                        X_nom[9*i:9*(i+1)] = self.params.X_nom[k][0:9]
            else:
                if not make_cyclic:
                    X_nom[9*i:9*(i+1)]  = self.params.X_nom[-1][0:9]
                    i += 1
                else:
                    # make this cyclic later
                    pass

        X_nom[0:9] = X_init
        X_ter = self.params.X_nom[-1][0:9]

        self.mp.create_bound_constraints(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(np.tile(self.params.W_X, self.horizon), self.params.W_X_ter, X_ter, X_nom)
        self.mp.create_cost_F(np.tile(self.params.W_F, self.horizon))

        ## IK Costs ##
        # adding contact costs
        for i in range(self.ik_horizon):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.params.cnt_wt,
                                                              "cnt_" + str(0) + self.eff_names[j], i)
        
        ft = t
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            # account for terminal here. 
            if ft <= self.params.x_reg[-1][-1]:
                for k in range(len(self.params.x_reg)):
                    if ft > self.params.wt_reg[k][1] and ft <= self.params.wt_reg[k][2]:
                        self.ik.add_state_regularization_cost_single(i, self.params.wt_reg[k][0], \
                                    "xReg", self.params.state_wt[k][0:self.rmodel.nq + self.rmodel.nv],\
                                                 self.x_reg, False)
            else:
                if not make_cyclic:
                    self.ik.add_state_regularization_cost_single(i, self.params.wt_reg[k][0], \
                                    "xReg", self.params.state_wt[k][0:self.rmodel.nq + self.rmodel.nv], \
                                                self.x_reg, False)
                    i += 1
                else:
                    # make this cyclic later
                    pass
