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
        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))
        
        # Set up constraints for Dynamics
        self.bx = 0.25
        self.by = 0.25
        self.bz = 0.25
        self.fx_max = 25.0
        self.fy_max = 25.0
        self.fz_max = 25.0

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
        ft = t - self.params.dt_arr[0]
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            ft = np.round(ft, 3)
            if ft < self.params.cnt_plan[-1][0][5]:
                for k in range(len(self.params.cnt_plan)):
                ## making an assumption that each phase is fixed for each leg
                ## can be changed
                    if ft >= self.params.cnt_plan[k][0][4] and ft < self.params.cnt_plan[k][0][5]:
                        for j in range(len(self.eff_names)):
                            ## not sure how to update the plan based on current foot step locations
                            ## should discuss this
                            self.cnt_plan[i][j] = self.params.cnt_plan[k][j][0:4]
                        break

            else:
                ## case where the t is larger than the plan horizon
                if not make_cyclic:
                    for j in range(len(self.eff_names)):
                        self.cnt_plan[i][j] = self.params.cnt_plan[-1][j][0:4]
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
            i += 1
        
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
        ft = t - self.params.dt_arr[0]
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            ft = np.round(ft, 3)
            if ft < self.params.X_nom[-1][-1]:
                for k in range(len(self.params.X_nom)):
                    if ft >= self.params.X_nom[k][9] and ft < self.params.X_nom[k][10]:
                        X_nom[9*i:9*(i+1)] = self.params.X_nom[k][0:9]
                        break
                if i == self.params.n_col - 1:
                    X_ter = X_nom[-9:]
            else:
                if not make_cyclic:
                    X_nom[9*i:9*(i+1)]  = self.params.X_ter
                    if i == self.params.n_col - 1:
                        X_ter = self.params.X_ter
                else:
                    # make this cyclic later
                    pass

            i += 1

        X_nom[0:9] = X_init

        self.mp.create_bound_constraints(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(np.tile(self.params.W_X, self.horizon), self.params.W_X_ter, X_ter, X_nom)
        self.mp.create_cost_F(np.tile(self.params.W_F, self.horizon))

        ## IK Costs ##
        self.dt_arr = np.zeros(self.ik_horizon)
        # adding contact costs
        for i in range(self.ik_horizon):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.params.cnt_wt,
                                                              "cnt_" + str(0) + self.eff_names[j], i)
        
        ## Adding swing costs
        ft = t - self.params.dt_arr[0]
        i = 0
        while i < self.ik_horizon:    
            ft += self.params.dt_arr[min(i,self.ik_horizon-1)]
            self.dt_arr[min(i,self.ik_horizon-1)] = self.params.dt_arr[min(i,self.ik_horizon-1)]
            ft = np.round(ft, 3)
            if ft < self.params.swing_wt[-1][0][5]:
                for k in range(len(self.params.swing_wt)):
                    if ft >= self.params.swing_wt[k][0][4] and ft < self.params.swing_wt[k][0][5]:
                        for j in range(len(self.eff_names)):
                            if self.params.swing_wt[k][j][0] > 0:
                                self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.params.swing_wt[k][j][1:4], self.params.swing_wt[k][j][0],
                                                              "swing_" + str(0) + self.eff_names[j], i)
                        break
            else:
                if not make_cyclic:
                    pass
            i += 1

        ## State regularization
        ft = t - self.params.dt_arr[0]
        i = 0
        while i < self.ik_horizon + 1:    
            ## is there a more effecient way to handle terminal states?
            ## TODO: have to make sure that dt is picked from the right index. 
            ft += self.params.dt_arr[min(i,self.ik_horizon-1)]
            self.dt_arr[min(i,self.ik_horizon-1)] = self.params.dt_arr[min(i,self.ik_horizon-1)]
            ft = np.round(ft, 3)
            if ft < self.params.state_reg[-1][-1]:
                for k in range(len(self.params.state_reg)):
                    if ft >= self.params.state_scale[k][1] and ft < self.params.state_scale[k][2]:
                        if i < self.params.n_col:
                            self.ik.add_state_regularization_cost_single(i, self.params.state_scale[k][0], \
                                        "xReg", self.params.state_wt[k][0:2*self.rmodel.nv],\
                                                    self.params.state_reg[k][0:self.rmodel.nq + self.rmodel.nv])
                        else:
                            # account for terminal here. 
                            self.ik.add_state_regularization_cost(0, i, self.params.state_scale[k][0], \
                                                                "xReg", self.params.state_wt[k][0:2*self.rmodel.nv],\
                                                                            self.params.state_reg[k][0:self.rmodel.nq + self.rmodel.nv], True)
                        break
            else:
                if not make_cyclic:
                    if i < self.params.n_col:
                        self.ik.add_state_regularization_cost_single(i, self.params.state_scale[-1][0], \
                                        "xReg", self.params.state_wt[-1][0:2*self.rmodel.nv], \
                                                    self.params.state_reg[-1][0:self.rmodel.nq + self.rmodel.nv])
                    else:
                        self.ik.add_state_regularization_cost(0, i, self.params.state_scale[-1][0], \
                                                                "xReg", self.params.state_wt[-1][0:2*self.rmodel.nv],\
                                                                            self.params.state_reg[-1][0:self.rmodel.nq + self.rmodel.nv], True)
                else:
                    # make this cyclic later
                    pass

            i += 1

        ## control regularization
        ft = t - self.params.dt_arr[0]
        i = 0
        while i < self.params.n_col + 1:    
            ft += self.params.dt_arr[min(i,self.params.n_col-1)]
            ft = np.round(ft, 3)
            if ft < self.params.ctrl_scale[-1][-1]:
                for k in range(len(self.params.ctrl_scale)):
                    if ft >= self.params.ctrl_scale[k][1] and ft < self.params.ctrl_scale[k][2]:
                        if i < self.params.n_col:
                            self.ik.add_ctrl_regularization_cost_single(i, self.params.ctrl_wt[k][0], \
                                        "ctrlReg", self.params.ctrl_wt[k][0:self.rmodel.nv],\
                                                    self.params.ctrl_reg[k][0:self.rmodel.nv])
                        else:
                            # account for terminal here. 
                            self.ik.add_ctrl_regularization_cost(0, i, self.params.ctrl_scale[k][0], \
                                                                "ctrlReg", self.params.ctrl_wt[k][0:self.rmodel.nv],\
                                                                            self.params.ctrl_reg[k][0:self.rmodel.nv], True)
                        break
            else:
                if not make_cyclic:
                    if i < self.params.n_col:
                        self.ik.add_ctrl_regularization_cost_single(i, self.params.ctrl_wt[-1][0], \
                                        "ctrlReg", self.params.ctrl_wt[-1][0:self.rmodel.nv],\
                                                    self.params.ctrl_reg[-1][0:self.rmodel.nv])
                    else:
                        self.ik.add_ctrl_regularization_cost(0, i, self.params.ctrl_scale[-1][0], \
                                                                "ctrlReg", self.params.ctrl_wt[-1][0:self.rmodel.nv],\
                                                                            self.params.ctrl_reg[-1][0:self.rmodel.nv], True)
                else:
                    # make this cyclic later
                    pass

            i += 1

        self.ik.setup_costs(self.dt_arr)

    def optimize(self, q, v, t, X_wm = None, F_wm = None, P_wm = None):
        """
        Generates full body trajectory
        Input:
            q : current joint configuration
            v : current joint velocity
            t : current time into the plan
        """
        t1 = time.time()
        self.create_contact_plan(q, v, t)
        #Creates costs for IK and Dynamics
        self.create_costs(q, v, t)

        t2 = time.time()

        self.kd.optimize(q, v, 80, 1)

        t3 = time.time()

        print("Cost Time :", t2 - t1)
        print("Solve Time : ", t3 - t2)
        print(" ================================== ")

        com_opt = self.mp.return_opt_com()
        mom_opt = self.mp.return_opt_mom()
        F_opt = self.mp.return_opt_f()
        xs = self.ik.get_xs()
        us = self.ik.get_us()

        n_eff = 3*len(self.eff_names)
        for i in range(len(xs)-2):
            if i == 0:
                self.f_int = np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt_arr[i]/0.001))
                self.xs_int = np.linspace(xs[i], xs[i+1], int(self.dt_arr[i]/0.001))
                self.us_int = np.linspace(us[i], us[i+1], int(self.dt_arr[i]/0.001))

                self.com_int = np.linspace(com_opt[i], com_opt[i+1], int(self.dt_arr[i]/0.001))
                self.mom_int = np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt_arr[i]/0.001))
            else:
                self.f_int =  np.vstack((self.f_int, np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt_arr[i]/0.001))))
                self.xs_int = np.vstack((self.xs_int, np.linspace(xs[i], xs[i+1], int(self.dt_arr[i]/0.001))))
                self.us_int = np.vstack((self.us_int, np.linspace(us[i], us[i+1], int(self.dt_arr[i]/0.001))))

                self.com_int = np.vstack((self.com_int, np.linspace(com_opt[i], com_opt[i+1], int(self.dt_arr[i]/0.001))))
                self.mom_int = np.vstack((self.mom_int, np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt_arr[i]/0.001))))

        return self.xs_int, self.us_int, self.f_int