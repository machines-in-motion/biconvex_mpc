## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import time
import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from gait_planner_cpp import GaitPlanner

from matplotlib import pyplot as plt

class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, dt, state_wt, x_reg, planning_time):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            dt : discretization
            state_wt : state regularization wt
            x_reg : joint config about which regulation is done
            plan_freq : planning frequency in seconds
            gait : which gait to generate
        """

        ## Note : only creates plan for horizon of 2*st
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        self.dt = dt
        self.foot_size = 0.018
        self.step_height = 0.1

        #TODO: DEPRECATE THIS...
        #Use for a fixed frequency planning time
        self.planning_time = planning_time

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))

        self.current_contact = np.zeros(4)

        self.wt = [1e4, 1e4]
        self.state_wt = state_wt
        self.x_reg = x_reg

        # --- Set up gait parameters ---
        # self.gait_period = 0.35
        # self.stance_percent = [0.55, 0.55, 0.55, 0.55]
        # self.gait_dt = 0.05
        # self.phase_offset = [0.0, 0.5, 0.5, 0.0]
        # self.gait_planner = GaitPlanner(self.gait_period, np.array(self.stance_percent), \
        #                                 np.array(self.phase_offset), self.step_height)

        self.gait_period = 0.20
        self.stance_percent = [1.0, 1.0, 1.0, 1.0]
        self.gait_dt = 0.05
        self.phase_offset = [0.0, 0.5, 0.5, 0.0]
        self.gait_planner = GaitPlanner(self.gait_period, np.array(self.stance_percent), \
                                        np.array(self.phase_offset), self.step_height)

        # --- Set up Inverse Kinematics ---
        self.ik = InverseKinematics(r_urdf, dt, self.gait_period)

        # --- Set up Dynamics ---
        self.m = pin.computeTotalMass(self.rmodel)
        self.rho = 5e+4 # penalty on dynamic constraint violation
        self.mp = BiConvexMP(self.m, self.dt, 2*self.gait_period, len(self.eff_names), rho = self.rho)

        #Different horizon parameterizations; only self.gait_horizon works for now
        self.gait_horizon = 2
        #self.fixed_horizon = fixed_horizon
        self.time_horizon = int(2.0/self.gait_dt)

        self.horizon = int(round(self.gait_horizon*self.gait_period/self.gait_dt))

        self.X_nom = np.zeros((9*self.horizon))

        # Set up Weights for Dynamics
        self.W_X = np.array([1e-5, 1e-5, 1e+5, 1e+2, 1e+2, 1e+2, 1e4, 1e4, 3e3])
        self.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e+3, 1e+3, 1e+3, 1e+5, 1e+5, 1e+5])
        self.W_F = np.array(4*[1e+1, 1e+1, 1e+1])

        # Set up constraints for Dynamics
        self.bx = 0.25
        self.by = 0.25
        self.bz = 0.25
        self.fx_max = 15
        self.fy_max = 15
        self.fz_max = 15

        # --- Set up other variables ---
        # For interpolation (should be moved to the controller)
        self.xs_int = np.zeros((len(self.x_reg), int(self.dt/0.001)))
        self.us_int = np.zeros((self.rmodel.nv, int(self.dt/0.001)))
        self.f_int = np.zeros((4*len(self.eff_names), int(self.dt/0.001)))

        # For plotting
        self.com_traj = []
        self.xs_traj = []
        self.q_traj = []
        self.v_traj = []

    def create_cnt_plan(self, q, v, t, next_loc, v_des):
        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
        self.prev_cnt = np.zeros((len(self.eff_names), 4))


        # Contact Plan Matrix: horizon x num_eef x 4: The '4' gives the contact plan and location:
        # i.e. the last vector should be [1/0, x, y, z] where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)

        for i in range(self.horizon):
            for j in range(len(self.eff_names)):
                if i == 0:
                    #First time step
                    if self.gait_planner.get_phase(t, j) == 1:
                        #If foot is currently in contact
                        self.cnt_plan[i][j][0] = 1
                        self.cnt_plan[i][j][1:4] = self.rdata.oMf[self.ee_frame_id[j]].translation
                        self.prev_cnt[j][0] = 1
                        self.prev_cnt[j][1:4] = self.cnt_plan[i][j][1:4].copy()
                    else:
                        #If foot is not in contact
                        self.cnt_plan[i][j][0] = 0
                        self.prev_cnt[j][0] = 0
                        self.prev_cnt[j][1:4] = next_loc[j]
                else:
                    #All other time steps
                    if self.gait_planner.get_phase(t + i*self.gait_dt, j) == 1:
                        #If foot will be in contact
                        self.cnt_plan[i][j][0] = 1
                        if self.prev_cnt[j][0] == 1:
                            #If I was in stance previously, use the same foot location
                            self.cnt_plan[i][j][1:4] = self.prev_cnt[j][1:4]
                        else:
                            # If I was in swing previously, use the raibert heuristic
                            # Raibert heuristic is based off the previous contact location
                            self.cnt_plan[i][j][1:4] = self.prev_cnt[j][1:4] + v_des*self.stance_percent[j]*self.gait_period/2
                            self.cnt_plan[i][j][3] = self.foot_size
                    else:
                        #If foot will not be in contact
                        self.cnt_plan[i][j][0] = 0
                        self.prev_cnt[j][0] = 0

        return self.cnt_plan

    def create_costs(self, q, v, v_des, t, wt_xreg, wt_ureg):
        """
        Input:
            q : joint positions at current time
            v : joint velocity at current time
            v_des : desired velocity of center of mass
            cnt_plan : contact plan
            sh : step height
            t : time within the step
        """

        self.x0 = np.hstack((q,v))

        # --- Set Up IK --- #
        for i in range(int(round(self.gait_period/self.gait_dt))):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    #If stance phase add tracking to current position
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.wt[0],
                                                              "cnt_" + str(0) + self.eff_names[j], i)
                else:
                    #If Not in stance phase

                    #Add tracking for next
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.wt[1],
                                                              "end_pos_" + str(0) + self.eff_names[j], i)
                    if self.gait_planner.get_percent_in_phase(t+i * self.gait_dt, j) < 0.5:
                        pos = self.cnt_plan[i][j][1:4] - v_des*()
                        pos[2] = self.step_height
                        self.ik.add_position_tracking_task_single(self.ee_frame_id[j], pos, self.wt[1],
                                                                  "via_" + str(0) + self.eff_names[j], i)

        self.ik.add_state_regularization_cost(0, self.gait_period, wt_xreg, "xReg", self.state_wt, self.x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, self.gait_period, wt_ureg, "uReg", False)

        self.ik.add_state_regularization_cost(0, self.gait_period, wt_xreg, "xReg", self.state_wt, self.x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, self.gait_period, wt_ureg, "uReg", True)


        self.ik.setup_costs()

        # --- Setup Dynamics --- #

        # initial and ter state
        self.X_init = np.zeros(9)
        pin.computeCentroidalMomentum(self.rmodel, self.rdata)
        self.X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
        self.X_init[3:] = np.array(self.rdata.hg)
        self.X_init[3:6] /= self.m

        self.X_nom[0::9] = self.X_init[0]
        self.X_nom[2::9] = 0.2
        self.X_nom[3::9] = v_des[0]
        self.X_nom[4::9] = v_des[1]
        self.X_nom[5::9] = v_des[2]

        amom = 0.8*self.compute_ori_correction(q, np.array([0,0,0,1]))
        self.X_nom[6::9] = amom[0]
        self.X_nom[7::9] = amom[1]
        self.X_nom[8::9] = amom[2]

        X_ter = np.zeros_like(self.X_init)
        X_ter[0:3] = self.X_init[0:3].copy()
        X_ter[2] = 0.2

        X_ter[0:2] = self.X_init[0:2] + (self.horizon*v_des*self.gait_dt)[0:2] #Changed this
        X_ter[3:6] = v_des
        X_ter[6:] = amom

        # Setup dynamic optimization
        self.mp.create_contact_array_2(np.array(self.cnt_plan))
        self.mp.create_bound_constraints_2(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(self.W_X, self.W_X_ter, X_ter, self.X_nom)
        self.mp.create_cost_F(self.W_F)

        #Shift costs & constraints (Assumes shift of one knot point for now...)
        # TODO: Make update_dynamics take in the time
        # self.mp.update_dynamics()

    def compute_ori_correction(self, q, des_quat):
        """
        This function computes the AMOM required to correct for orientation
        q : current joint configuration
        des_quat : desired orientation
        """
        pin_quat = pin.Quaternion(np.array(q[3:7]))
        pin_des_quat = pin.Quaternion(np.array(des_quat))

        omega = pin.log3((pin_des_quat*(pin_quat.inverse())).toRotationMatrix())

        return omega

    def optimize(self, q, v, t, next_loc, v_des, step_height, x_reg, u_reg, current_contact):

        #TODO: Move to C++
        t1 = time.time()
        self.current_contact = current_contact
        self.create_cnt_plan(q, v, t, next_loc, v_des)

        #Creates costs for IK and Dynamics
        self.create_costs(q, v, v_des, t, x_reg, u_reg)
        self.q_traj.append(q)
        self.v_traj.append(v)

        # --- Dynamics optimization ---

        # Optimize Dynamics
        t2 = time.time()
        com_opt, F_opt, mom_opt = self.mp.optimize(self.X_init, 50)
        t3 = time.time()

        self.com_traj.append(com_opt)

        # --- IK Optimization ---

        # Add tracking costs from Dynamic optimization
        self.ik.add_centroidal_momentum_tracking_task(0, self.gait_period, mom_opt[0:int(round(self.gait_period/self.dt))], 1e2, "mom_track", False)
        self.ik.add_centroidal_momentum_tracking_task(0, self.gait_period, mom_opt[int(round(self.gait_period/self.dt))], 1e1, "mom_track", True)

        self.ik.add_com_position_tracking_task(0, self.gait_period, com_opt[0:int(round(self.gait_period/self.dt))], 1e4, "com_track_cost", False)
        self.ik.add_com_position_tracking_task(0, self.gait_period, com_opt[int(round(self.gait_period/self.dt))], 1e5, "com_track_cost", True)

        t4 = time.time()
        self.ik.optimize(np.hstack((q,v)))
        t5 = time.time()
        # print("cost", t2 - t1)
        # print("dyn", t3 - t2)
        # print("ik", t5 - t4)
        # print("total", t5 - t1)
        # print("------------------------")
        xs = self.ik.get_xs()
        us = self.ik.get_us()
        self.xs_traj.append(xs)

        n_eff = 3*len(self.eff_names)
        ind = int(self.planning_time/self.dt) + 1 # 1 is to account for time lag
        for i in range(ind):
            if i == 0:
                self.f_int = np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))
                self.xs_int = np.linspace(xs[i], xs[i+1], int(self.dt/0.001))
                self.us_int = np.linspace(us[i], us[i+1], int(self.dt/0.001))

                self.com_int = np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))
                self.mom_int = np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))
            else:
                self.f_int =  np.vstack((self.f_int, np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))))
                self.xs_int = np.vstack((self.xs_int, np.linspace(xs[i], xs[i+1], int(self.dt/0.001))))
                self.us_int = np.vstack((self.us_int, np.linspace(us[i], us[i+1], int(self.dt/0.001))))

                self.com_int = np.vstack((self.com_int, np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))))
                self.mom_int = np.vstack((self.mom_int, np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))))

        return self.xs_int, self.us_int, self.f_int

    def reset(self):
        self.ik = InverseKinematics(self.r_urdf, self.dt, self.gait_period)
        self.mp = BiConvexMP(self.m, self.dt, 2*self.gait_period, len(self.eff_names), rho = self.rho)

    def plot(self, q_real=None):
        self.com_traj = np.array(self.com_traj)
        print(self.com_traj.shape)
        self.q_traj = np.array(self.q_traj)
        self.v_traj = np.array(self.v_traj)
        x = self.dt*np.arange(0, len(self.com_traj[1]) + int((self.planning_time/self.dt))*len(self.com_traj), 1)
        if q_real:
            q_real = np.array(q_real)[::int(self.dt/0.001)]

        # com plots
        fig, ax = plt.subplots(3,1)
        for i in range(0, len(self.com_traj)):
            st_hor = i*int(self.plan_freq/self.dt)

            if i == 0:
                com = pin.centerOfMass(self.rmodel, self.rdata, self.q_traj[i], self.v_traj[i])
                ax[0].plot(x[st_hor], com[0], "o", label = "real com x")
                ax[1].plot(x[st_hor], com[1], "o", label = "real com y")
                ax[2].plot(x[st_hor], com[2], "o", label = "real com z")


                ax[0].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,0], label = "com x")
                ax[1].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,1], label = "com y")
                ax[2].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,2], label = "com z")

            else:
                com = pin.centerOfMass(self.rmodel, self.rdata, self.q_traj[i], self.v_traj[i])
                ax[0].plot(x[st_hor], com[0], "o")
                ax[1].plot(x[st_hor], com[1], "o")
                ax[2].plot(x[st_hor], com[2], "o")

                ax[0].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,0])
                ax[1].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,1])
                ax[2].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,2])

        ax[0].grid()
        ax[0].legend()

        ax[1].grid()
        ax[1].legend()

        ax[2].grid()
        ax[2].legend()

        plt.show()

    def plot_joints(self):
        self.xs_traj = np.array(self.xs_traj)
        self.xs_traj = self.xs_traj[:,:,:self.rmodel.nq]
        self.q_traj = np.array(self.q_traj)
        x = self.dt*np.arange(0, len(self.xs_traj[1]) + len(self.xs_traj), 1)
        # com plots
        fig, ax = plt.subplots(3,1)
        for i in range(len(self.xs_traj)):
            st_hor = i*int(self.plan_freq/self.dt)
            ax[0].plot(x[st_hor], self.q_traj[i][10], 'o')
            ax[0].plot(x[st_hor:st_hor + len(self.xs_traj[i])], self.xs_traj[i][:,10])

        plt.show()

    def plot_plan(self):
        xs = self.ik.get_xs()
        opt_mom = np.zeros((len(xs), 6))
        opt_com = np.zeros((len(xs), 3))
        for i in range(len(xs)):
            q = xs[i][:self.rmodel.nq]
            v = xs[i][self.rmodel.nq:]
            pin.forwardKinematics(self.rmodel, self.rdata, q, v)
            pin.computeCentroidalMomentum(self.rmodel, self.rdata)
            opt_com[i] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
            opt_mom[i] = np.array(self.rdata.hg)
            opt_mom[i][0:3] /= self.m

        self.mp.add_ik_com_cost(opt_com)
        self.mp.add_ik_momentum_cost(opt_mom)

        self.mp.stats()













