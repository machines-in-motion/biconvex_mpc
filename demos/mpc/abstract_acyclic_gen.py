## This file creates the costs and contact planner for the kino-dyn planner
## Author : Avadesh Meduri, Paarth Shah
## Date : 23/09/2021

import time
import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from biconvex_mpc_cpp import KinoDynMP

from matplotlib import pyplot as plt

class SoloAcyclicGen:

    def __init__(self, robot, r_urdf):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
        """
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        # --- Set up Dynamics ---
        self.m = pin.computeTotalMass(self.rmodel)

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.n_eff = 4
        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))
        
        # Set up constraints for Dynamics
        self.fx_max = 25.0
        self.fy_max = 25.0
        self.fz_max = 25.0

        # Set up optional parameters for updating contact plan
        self.use_current_eef_location = False
        self.use_current_contact = False

    def update_motion_params(self, weight_abstract, q0, t0):
        """
        Updates the gaits
        Input:
            weight_abstract : the parameters of the gaits
            q0 : joint config to bias 
            t : time to bias
        """
        self.q0 = q0
        self.t0 = t0
        self.params = weight_abstract
        self.freq = self.params.plan_freq[0][0]
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
            self.size += 1

        self.xs_int = np.zeros((self.rmodel.nq + self.rmodel.nv, self.size))
        self.us_int = np.zeros((self.rmodel.nv, self.size))
        self.f_int = np.zeros((4*len(self.eff_names), self.size))

    def create_contact_plan(self, q, v, t, make_cyclic = False):
        """
        Creates contact plan based on moving horizon
        Input:
            q : current joint configurationself.dt_arr
            v : current joint velocity
            t : current time into the plan
        """
        self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
        ft = np.round( (t - self.params.dt_arr[0] - self.t0),3)

        prev_current_eef_used = np.ones(len(self.eff_names))

        for i in range(self.params.n_col):
            ft += np.round(self.params.dt_arr[i],3)

            if ft < self.params.cnt_plan[-1][0][5]:
                for k in range(len(self.params.cnt_plan)):
                    if ft >= self.params.cnt_plan[k][0][4] and ft < self.params.cnt_plan[k][0][5]:
                        for j in range(len(self.eff_names)):
                            self.cnt_plan[i][j] = self.params.cnt_plan[k][j][0:4]

                            # if self.use_current_eef_location and cnt_plan[i][j][0] == 1 and \
                            #     prev_current_eef_used[j] == 1:
                            #         cnt_plan[i][j][1:4] = eef_locations[j]

                            # if cnt_plan[i][j][0] == 0:
                            #     prev_current_eef_used[j] = 0
                        break
            else:
                if not make_cyclic:
                    for j in range(len(self.eff_names)):
                        self.cnt_plan[i][j] = self.params.cnt_plan[-1][j][0:4]

                        # if self.use_current_eef_location and cnt_plan[i][j][0] == 1 and \
                        #     prev_current_eef_used[j] == 1:
                        #     cnt_plan[i][j][1:4] = eef_locations[j]

                else:
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

        ## Dynamics Costs ##
        X_nom = np.zeros((9*self.horizon))
        ft = t - self.params.dt_arr[0] - self.t0
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

        self.bounds = np.zeros((self.horizon, 6))
        ft = t - self.params.dt_arr[0] - self.t0
        i = 0
        while i < self.params.n_col:    
            ft += self.params.dt_arr[i]
            ft = np.round(ft, 3)
            if ft < self.params.bounds[-1][-1]:
                for k in range(len(self.params.bounds)):
                    if ft >= self.params.bounds[k][-2] and ft < self.params.bounds[k][-1]:
                        self.bounds[i] = self.params.bounds[k][0:6]
                        break
            else:
                if not make_cyclic:
                    self.bounds[i] = self.params.bounds[-1][0:6]
                else:
                    # make this cyclic later
                    pass

            i += 1

        X_nom[0:9] = X_init

        self.mp.create_bound_constraints(self.bounds, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(np.tile(self.params.W_X, self.horizon), self.params.W_X_ter, X_ter, X_nom)
        self.mp.create_cost_F(np.tile(self.params.W_F, self.horizon))

        ## IK Costs ##
        # Fix this
        self.dt_arr = np.zeros(self.ik_horizon+1)

        # adding contact costs
        for i in range(self.ik_horizon):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.params.cnt_wt,
                                                              "cnt_" + str(0) + self.eff_names[j], i)

        ## Adding swing costs
        if isinstance(self.params.swing_wt, np.ndarray) or isinstance(self.params.swing_wt, list):
            ft = t - self.params.dt_arr[0] - self.t0
            i = 0
            while i < self.ik_horizon:    
                ft += self.params.dt_arr[min(i, self.ik_horizon-1)]
                ft = np.round(ft, 3)
                if ft < self.params.swing_wt[-1][0][5]:
                    for k in range(len(self.params.swing_wt)):
                        if self.params.swing_wt[k][0][4] <= ft < self.params.swing_wt[k][0][5]:
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
        ft = t - self.params.dt_arr[0] - self.t0
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

        # control regularization
        ft = t - self.params.dt_arr[0] - self.t0
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
        pin.forwardKinematics(self.rmodel, self.rdata, q, np.zeros(self.rmodel.nv))
        pin.updateFramePlacements(self.rmodel, self.rdata)
        pin.framesForwardKinematics(self.rmodel, self.rdata, q)

        t1 = time.time()        
        # q[0:2] -= self.q0[0:2] - [0.2,0] 
        
        self.create_contact_plan(q, v, t)
        #Creates costs for IK and Dynamics
        self.create_costs(q, v, t)

        t2 = time.time()

        self.kd.optimize(q, v, 50, 1)

        t3 = time.time()

        print("Cost Time :", t2 - t1)
        print("Solve Time : ", t3 - t2)
        print(" ================================== ")

        optimized_forces = self.mp.return_opt_f()
        xs = self.ik.get_xs()
        us = self.ik.get_us()

        #Interpolation of optimized variables
        n_eff = 3*len(self.eff_names)
        for i in range(len(xs)):
            if i == 0:
                self.f_int = np.linspace(optimized_forces[i*n_eff:n_eff*(i+1)], optimized_forces[n_eff*(i):n_eff*(i+1)], int(self.dt_arr[i]/0.001))
                self.xs_int = np.linspace(xs[i], xs[i], int(self.dt_arr[i]/0.001))
                self.us_int = np.linspace(us[i], us[i], int(self.dt_arr[i]/0.001))

            elif i == len(xs)-1:
                self.xs_int = np.vstack((self.xs_int, np.linspace(xs[i], xs[i], int(self.dt_arr[i]/0.001))))
            
            else:
                self.f_int =  np.vstack((self.f_int, np.linspace(optimized_forces[i*n_eff:n_eff*(i+1)], optimized_forces[n_eff*(i):n_eff*(i+1)], int(self.dt_arr[i]/0.001))))
                self.xs_int = np.vstack((self.xs_int, np.linspace(xs[i], xs[i], int(self.dt_arr[i]/0.001))))
                self.us_int = np.vstack((self.us_int, np.linspace(us[i], us[i], int(self.dt_arr[i]/0.001))))

        return self.xs_int, self.us_int, self.f_int

    def get_plan_freq(self, t):
        """
        This function returns the planning frequency based on the plan
        """
        for k in range(len(self.params.plan_freq)):
            if t-self.t0 < self.params.plan_freq[-1][-1]:
                if t-self.t0 < self.params.plan_freq[k][-1] and t-self.t0 >= self.params.plan_freq[k][-2]:
                    return self.params.plan_freq[k][0]
            else:
                return self.params.plan_freq[-1][0]

    def get_gains(self, t):
        """
        returns the gains for the ID controller at different times based on the plan
        """
        for k in range(len(self.params.kp)):
            if t-self.t0< self.params.kp[-1][-1]:
                if t-self.t0 < self.params.kp[k][-1] and t-self.t0 >= self.params.kp[k][-2]:
                    return self.params.kp[k][0], self.params.kd[k][0]
            else:
                return self.params.kp[-1][0], self.params.kd[-1][0]

    def save_plan(self, file_name):
        """
        This function saves the plan for later plotting
        Input:
            file_name : name of the file
        """

        np.savez("./"+file_name, com_opt = self.mp.return_opt_com(),\
                                 mom_opt = self.mp.return_opt_mom(),\
                                 F_opt = self.mp.return_opt_f(), \
                                 ik_com_opt = self.ik.return_opt_com(),\
                                 ik_mom_opt = self.ik.return_opt_mom(),\
                                 xs = self.ik.get_xs())
                                 
        print("finished saving ...")
        assert False

    def plot(self, q, v, plot_force = True):
        com_opt = self.mp.return_opt_com()
        mom_opt = self.mp.return_opt_mom()
        optimized_forces = self.mp.return_opt_f()
        ik_com_opt = self.ik.return_opt_com()
        ik_mom_opt = self.ik.return_opt_mom()
        com = pin.centerOfMass(self.rmodel, self.rdata, q.copy(), v.copy())

        # Plot Center of Mass
        fig, ax = plt.subplots(3,1)
        ax[0].plot(com_opt[:, 0], label="Dyn com x")
        ax[0].plot(ik_com_opt[:, 0], label="IK com x")
        ax[0].plot(com[0], 'o', label="Current Center of Mass x")
        ax[1].plot(com_opt[:, 1], label="Dyn com y")
        ax[1].plot(ik_com_opt[:, 1], label="IK com y")
        ax[1].plot(com[1], 'o', label="Current Center of Mass y")
        ax[2].plot(com_opt[:, 2], label="Dyn com z")
        ax[2].plot(ik_com_opt[:, 2], label="IK com z")
        ax[2].plot(com[2], 'o', label="Current Center of Mass z")

        ax[0].grid()
        ax[0].legend()
        ax[1].grid()
        ax[1].legend()
        ax[2].grid()
        ax[2].legend()

        # Plot End-Effector Forces
        if plot_force:
            fig, ax_f = plt.subplots(self.n_eff, 1)
            for n in range(self.n_eff):
                ax_f[n].plot(optimized_forces[3*n::3*self.n_eff], label = self.eff_names[n] + " Fx")
                ax_f[n].plot(optimized_forces[3*n+1::3*self.n_eff], label = self.eff_names[n] + " Fy")
                ax_f[n].plot(optimized_forces[3*n+2::3*self.n_eff], label = self.eff_names[n] + " Fz")
                ax_f[n].grid()
                ax_f[n].legend()

        # Plot Momentum
        fig, ax_m = plt.subplots(6,1)
        ax_m[0].plot(mom_opt[:, 0], label = "Dyn linear_momentum x")
        ax_m[0].plot(ik_mom_opt[:, 0], label="IK linear_momentum x")
        ax_m[1].plot(mom_opt[:, 1], label = "linear_momentum y")
        ax_m[1].plot(ik_mom_opt[:, 1], label="Dyn IK linear_momentum y")
        ax_m[2].plot(mom_opt[:, 2], label = "linear_momentum z")
        ax_m[2].plot(ik_mom_opt[:, 2], label="Dyn IK linear_momentum z")
        ax_m[3].plot(mom_opt[:, 3], label = "Dyn Angular momentum x")
        ax_m[3].plot(ik_mom_opt[:, 3], label="IK Angular momentum x")
        ax_m[4].plot(mom_opt[:, 4], label = "Dyn Angular momentum y")
        ax_m[4].plot(ik_mom_opt[:, 4], label="IK Angular momentum y")
        ax_m[5].plot(mom_opt[:, 5], label = "Dyn Angular momentum z")
        ax_m[5].plot(ik_mom_opt[:, 5], label="IK Angular momentum z")
        ax_m[0].grid()
        ax_m[0].legend()
        ax_m[1].grid()
        ax_m[1].legend()
        ax_m[2].grid()
        ax_m[2].legend()
        ax_m[3].grid()
        ax_m[3].legend()
        ax_m[4].grid()
        ax_m[4].legend()
        ax_m[5].grid()
        ax_m[5].legend()

        # Plot Linear Momentum
        fig, ax_am = plt.subplots(3,1)
        ax_am[0].plot(mom_opt[:, 3], label = "Dynamics Angular Momentum around X")
        ax_am[0].plot(ik_mom_opt[:, 3], label="Kinematic Angular Momentum around X")
        ax_am[1].plot(mom_opt[:, 4], label = "Dynamics Angular Momentum around Y")
        ax_am[1].plot(ik_mom_opt[:, 4], label="Kinematic Angular Momentum around Y")
        ax_am[2].plot(mom_opt[:, 5], label = "Dynamics Angular Momentum around Z")
        ax_am[2].plot(ik_mom_opt[:, 5], label="Kinematic Angular Momentum around Z")
        ax_am[0].grid()
        ax_am[0].legend()

        ax_am[1].grid()
        ax_am[1].legend()

        ax_am[2].grid()
        ax_am[2].legend()

        plt.show()