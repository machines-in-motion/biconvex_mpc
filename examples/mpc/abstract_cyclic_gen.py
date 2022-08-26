## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri & Paarth Shah
## Date : 6/05/2021

import time
import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from biconvex_mpc_cpp import BiconvexMP, KinoDynMP
from gait_planner_cpp import GaitPlanner
from matplotlib import pyplot as plt

class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, x_reg, planning_time, q0, height_map = None):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            dt : discretization
            x_reg : joint config about which regulation is done
            plan_freq : planning frequency in seconds
        """

        ## Note : only creates plan for horizon of 2*st
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        self.foot_size = 0.018

        #TODO: DEPRECATE THIS...
        #Use for a fixed frequency planning time
        self.planning_time = planning_time

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.hip_names = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]
        self.n_eff = 4

        pin.forwardKinematics(self.rmodel, self.rdata, q0, np.zeros(self.rmodel.nv))
        pin.updateFramePlacements(self.rmodel, self.rdata)
        com_init = pin.centerOfMass(self.rmodel, self.rdata, q0, np.zeros(self.rmodel.nv))

        pin.framesForwardKinematics(self.rmodel, self.rdata, q0)
        pin.crba(self.rmodel, self.rdata, q0)
        self.I_composite_b = self.rdata.Ycrb[1].inertia

        self.gravity = 9.81

        self.offsets = np.zeros((len(self.eff_names), 3))
        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))
            self.offsets[i] = self.rdata.oMf[self.rmodel.getFrameId(self.hip_names[i])].translation - com_init.copy()
            self.offsets[i] = np.round(self.offsets[i], 3)

        # Contact-planning offsets
        self.offsets[0][0] -= 0.00 #Front Left_X
        self.offsets[0][1] += 0.04 #Front Left_Y

        self.offsets[1][0] -= 0.00 #Front Right_X
        self.offsets[1][1] -= 0.04 #Front Right_Y

        self.offsets[2][0] += 0.00 #Hind Left_X
        self.offsets[2][1] += 0.04 #Hind Left_Y

        self.offsets[3][0] += 0.00 #Hind Right X
        self.offsets[3][1] -= 0.04 #Hind Right Y
        self.apply_offset = True

        #Rotate offsets to local frame
        R = pin.Quaternion(np.array(q0[3:7])).toRotationMatrix()
        for i in range(len(self.eff_names)):
            #Rotate offsets to local frame
            self.offsets[i] = np.matmul(R.T, self.offsets[i])

        #Current Contact
        self.x_reg = x_reg

        # --- Set up Dynamics ---
        self.m = pin.computeTotalMass(self.rmodel)

        #Set up logging for average optimization time
        self.dyn_comp_ave = 0.0
        self.dyn_comp_total = 0.0
        self.ik_comp_ave = 0.0
        self.ik_comp_total = 0.0
        self.num_optimization_ctr = 0 #Counter

        # Set up constraints for Dynamics
        self.bx = 0.45
        self.by = 0.45
        self.bz = 0.45
        self.fx_max = 15.0
        self.fy_max = 15.0
        self.fz_max = 15.0

        # For plotting
        self.com_traj = []
        self.xs_traj = []
        self.q_traj = []
        self.v_traj = []

        #Height Map (for contacts)
        self.height_map = height_map

    def update_gait_params(self, weight_abstract, t, ik_hor_ratio = 0.5):
        """
        Updates the gaits
        Input:
            weight_abstract : the parameters of the gaits
            t : time
            ik_hor_ratio : ik horion/dyn horizon
        """
        self.params = weight_abstract
        # --- Set up gait parameters ---
        self.gait_planner = GaitPlanner(self.params.gait_period, np.array(self.params.stance_percent), \
                                        np.array(self.params.phase_offset), self.params.step_ht)
        #Different horizon parameterizations; only self.params.gait_horizon works for now
        self.gait_horizon = self.params.gait_horizon
        self.horizon = int(np.round(self.params.gait_horizon*self.params.gait_period/self.params.gait_dt,2))

        # --- Set up Inverse Kinematics ---
        self.ik_horizon = int(np.round(ik_hor_ratio*self.params.gait_horizon*self.params.gait_period/self.params.gait_dt, 3))
        self.dt_arr = np.zeros(self.horizon)

        # kino dyn
        self.kd = KinoDynMP(self.r_urdf, self.m, len(self.eff_names), self.horizon, self.ik_horizon)
        self.kd.set_com_tracking_weight(self.params.cent_wt[0])
        self.kd.set_mom_tracking_weight(self.params.cent_wt[1])

        self.ik = self.kd.return_ik()
        self.mp = self.kd.return_dyn()

        self.mp.set_rho(self.params.rho)

        # --- Set up other variables ---
        self.X_nom = np.zeros((9*self.horizon))
        # For interpolation (should be moved to the controller)
        self.size = min(self.ik_horizon, int(self.planning_time/self.params.gait_dt) + 2)
        if self.planning_time > self.params.gait_dt:
            self.size -= 1
        self.xs_int = np.zeros((len(self.x_reg), self.size))
        self.us_int = np.zeros((self.rmodel.nv, self.size))
        self.f_int = np.zeros((4*len(self.eff_names), self.size))

        #Terminal cost
        self.simulation_step = 50
        self.terminal_cost = np.zeros((self.simulation_step, 9, 9))
        self.terminal_state = np.zeros((self.simulation_step, 9))
        self.terminal_hessian = np.zeros((self.simulation_step, 9, 9))
        self.terminal_gradient = np.zeros((self.simulation_step, 9))

    def create_cnt_plan(self, q, v, t, v_des, w_des):
        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        com = np.round(pin.centerOfMass(self.rmodel, self.rdata, q, v)[0:2], 3)
        z_height = pin.centerOfMass(self.rmodel, self.rdata, q, v)[2]
        vcom = np.round(v[0:3], 3)

        #Get current rotation
        R = pin.Quaternion(np.array(q[3:7])).toRotationMatrix()
        rpy_vector = pin.rpy.matrixToRpy(R)
        rpy_vector[0] = 0.0
        rpy_vector[1] = 0.0
        R = pin.rpy.rpyToMatrix(rpy_vector)

        vtrack = v_des[0:2] # this effects the step location (if set to vcom it becomes raibert)
        #vtrack = vcom[0:2]

        self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
        # This array determines when the swing foot cost should be enforced in the ik
        self.swing_time = np.zeros((self.horizon, len(self.eff_names)))
        self.prev_cnt = np.zeros((len(self.eff_names), 3))
        self.curr_cnt = np.zeros(len(self.eff_names))
        # Contact Plan Matrix: horizon x num_eef x 4: The '4' gives the contact plan and location:
        # i.e. the last vector should be [1/0, x, y, z] where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)

        for i in range(self.horizon):
            for j in range(len(self.eff_names)):
                if i == 0:
                    if self.gait_planner.get_phase(t, j) == 1:
                        self.cnt_plan[i][j][0] = 1
                        self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)
                        self.prev_cnt[j] = self.cnt_plan[i][j][1:4]
                    else:
                        self.cnt_plan[i][j][0] = 0
                        self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)
                        #     self.cnt_plan[i][j][1:3] += self.offsets[j]

                else:
                    #All other time steps
                    ft = np.round(t + i*self.params.gait_dt,3)

                    if self.gait_planner.get_phase(ft, j) == 1:
                        #If foot will be in contact
                        self.cnt_plan[i][j][0] = 1

                        if self.cnt_plan[i-1][j][0] == 1:
                            self.cnt_plan[i][j][1:4] = self.cnt_plan[i-1][j][1:4]
                        else:
                            hip_loc = com + np.matmul(R, self.offsets[j])[0:2] + i*self.params.gait_dt*vtrack
                            raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - 0.05*(vtrack - v_des[0:2])
                            ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                            ang_step = np.cross(ang_step, [0.0, 0.0, w_des])

                            self.cnt_plan[i][j][1:3] = raibert_step[0:2] + hip_loc + ang_step[0:2]

                            if self.height_map != None:
                                self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) +\
                                                                                self.foot_size
                            else:
                                self.cnt_plan[i][j][3] = self.foot_size

                        self.prev_cnt[j] = self.cnt_plan[i][j][1:4]

                    else:
                        #If foot will not be in contact
                        self.cnt_plan[i][j][0] = 0
                        per_ph = np.round(self.gait_planner.get_percent_in_phase(ft, j), 3)
                        hip_loc = com + np.matmul(R,self.offsets[j])[0:2] + i*self.params.gait_dt*vtrack
                        ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                        ang_step = np.cross(ang_step, [0.0, 0.0, w_des])

                        if per_ph < 0.5:
                            self.cnt_plan[i][j][1:3] = hip_loc + ang_step[0:2]
                        else:
                            raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - 0.05*(vtrack - v_des[0:2])
                            self.cnt_plan[i][j][1:3] = hip_loc + ang_step[0:2]

                        #What is this?
                        if per_ph - 0.5 < 0.02:
                            self.swing_time[i][j] = 1

                        if self.height_map != None:
                            self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) + \
                                                    self.foot_size
                        else:
                            self.cnt_plan[i][j][3] = self.foot_size

            if i == 0:
                dt = self.params.gait_dt - np.round(np.remainder(t,self.params.gait_dt),2)
                if dt == 0:
                    dt = self.params.gait_dt
            else:
                dt = self.params.gait_dt
            self.mp.set_contact_plan(self.cnt_plan[i], dt)
            self.dt_arr[i] = dt

        return self.cnt_plan

    def create_costs(self, q, v, v_des, w_des, ori_des):
        """
        Input:
            q : joint positions at current time
            v : joint velocity at current time
            v_des : desired velocity of center of mass
            t : time within the step
        """

        self.x0 = np.hstack((q,v))

        # --- Set Up IK --- #
        #Right now this is only setup to go for the *next* gait period only
        for i in range(self.ik_horizon):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.params.swing_wt[0],
                                                              "cnt_" + str(0) + self.eff_names[j], i)
                elif self.swing_time[i][j] == 1:
                    pos = self.cnt_plan[i][j][1:4].copy()
                    pos[2] = self.params.step_ht
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], pos, self.params.swing_wt[1],
                                                              "via_" + str(0) + self.eff_names[j], i)

        self.ik.add_state_regularization_cost(0, self.ik_horizon, self.params.reg_wt[0], "xReg", self.params.state_wt, self.x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, self.ik_horizon, self.params.reg_wt[1], "uReg", self.params.ctrl_wt, np.zeros(self.rmodel.nv), False)

        self.ik.add_state_regularization_cost(0, self.ik_horizon, self.params.reg_wt[0], "xReg", self.params.state_wt, self.x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, self.ik_horizon, self.params.reg_wt[1], "uReg", self.params.ctrl_wt, np.zeros(self.rmodel.nv), True)

        self.ik.setup_costs(self.dt_arr[0:self.ik_horizon])

        # --- Setup Dynamics --- #

        # initial and terminal state
        self.X_init = np.zeros(9)
        X_ter = np.zeros_like(self.X_init)
        pin.computeCentroidalMomentum(self.rmodel, self.rdata)
        self.X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q.copy(), v.copy())
        self.X_init[3:] = np.array(self.rdata.hg)
        self.X_init[3:6] /= self.m

        self.X_nom[0::9] = self.X_init[0]
        for i in range(1, self.horizon):
            self.X_nom[9*i+0] = self.X_nom[9*(i-1)+0] + v_des[0]*self.dt_arr[i]
            self.X_nom[9*i+1] = self.X_nom[9*(i-1)+1] + v_des[1]*self.dt_arr[i]

        self.X_nom[2::9] = self.params.nom_ht
        self.X_nom[3::9] = v_des[0]
        self.X_nom[4::9] = v_des[1]
        self.X_nom[5::9] = v_des[2]

        #Compute angular momentum / orientation correction
        R = pin.Quaternion(np.array(ori_des)).toRotationMatrix()
        rpy_vector = pin.rpy.matrixToRpy(R)
        rpy_vector[0] = 0.0
        rpy_vector[1] = 0.0
        des_quat = pin.Quaternion(pin.rpy.rpyToMatrix(rpy_vector))

        amom = self.compute_ori_correction(q, des_quat.coeffs())

        #Set terminal references
        X_ter[0:2] = self.X_init[0:2] + (self.params.gait_horizon*self.params.gait_period*v_des)[0:2] #Changed this
        X_ter[2] = self.params.nom_ht
        X_ter[3:6] = v_des
        X_ter[6:] = amom

        self.X_nom[6::9] = amom[0]*self.params.ori_correction[0]
        self.X_nom[7::9] = amom[1]*self.params.ori_correction[1]

        if w_des == 0:
            self.X_nom[8::9] = amom[2]*self.params.ori_correction[2]
        else:
            yaw_momentum = np.matmul(self.I_composite_b,[0.0, 0.0, w_des])[2]
            self.X_nom[8::9] = yaw_momentum
            X_ter[8] = yaw_momentum
            #print(yaw_momentum)

            # Setup dynamic optimization costs
        bounds = np.tile([-self.bx, -self.by, 0, self.bx, self.by, self.bz], (self.horizon,1))
        self.mp.create_bound_constraints(bounds, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(np.tile(self.params.W_X, self.horizon), self.params.W_X_ter, X_ter, self.X_nom)
        self.mp.create_cost_F(np.tile(self.params.W_F, self.horizon))

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

    def skew_symmetric(self, v):
        skew = np.zeros((3,3))
        skew[0,1] = -v[2]
        skew[0,2] = v[1]
        skew[1,0] = v[2]
        skew[1,2] = -v[0]
        skew[2,0] = -v[1]
        skew[2,1] = v[0]
        return skew


    def linearized_centroidal_dynamics(self, com, mom, force, time_index):
        A = np.identity(9)
        B = np.zeros((9, 3*len(self.eff_names)))
        # Bx = np.zeros((9, 9, 3*len(self.eff_names)))
        A[0:3, 3:6] = self.params.gait_dt * np.identity(3)
        sum_skew_force = np.zeros((3,3))
        for i in range(len(self.eff_names)):
            # print("force", force)
            sum_skew_force += self.skew_symmetric(force[3*i : 3*(i+1)])
            B[3:6, 3*i:3*(i+1)] +=  np. identity(3) * self.cnt_plan[time_index][i][0] * \
                                    self.params.gait_dt/self.m
            B[6:9, 3*i:3*(i+1)] += self.cnt_plan[time_index][i][0] * \
                    self.skew_symmetric(self.cnt_plan[time_index][i][1:4]-com) * self.params.gait_dt
            # Bx[0, 6:9, 3*i:3*(i+1)] = self.cnt_plan[time_index][i][0] * \
            #                           np.array([[0., 0., 0.],[0. ,0. , -1.],[0., 1., 0.]])
            # Bx[1, 6:9, 3*i:3*(i+1)] = self.cnt_plan[time_index][i][0] * \
            #                           np.array([[0., 0., 1.],[0. ,0. , 0.],[-1., 0., 0.]])
            # Bx[2, 6:9, 3*i:3*(i+1)] = self.cnt_plan[time_index][i][0] * \
                                      # np.array([[0., -1., 0.],[1. ,0. , 0.],[0., 0., 0.]])
        A[6:9, 0:3] = sum_skew_force
        return A, B


    # def compute_optimal_dyn_cost_ricatti(self, index, com_opt, mom_opt, F_opt):
    #     num_eff = len(self.eff_names)
    #     i = self.horizon-1
    #     P = np.diag(self.params.W_X_ter)
    #     while i >= self.horizon - index:
    #         print("i_ricatti:", i)
    #         # print("optimal_force:", F_opt[3*num_eff*i : 3*num_eff*(i+1)])
    #         # print("optimal_momentum:", mom_opt[i])
    #         # print("optimal com:", com_opt[i])
    #         A, B = self.linearized_centroidal_dynamics(com_opt[i], mom_opt[i],\
    #          F_opt[3*num_eff*i : 3*num_eff*(i+1)], i)
    #         inv_term = np.linalg.inv(np.diag(self.params.W_F) + B.T @ P @ B)
    #         P = np.diag(self.params.W_X) + A.T @ P @ A - A.T @ P @ B @ inv_term @ B.T @ P @ A
    #         # print("A:", A)
    #         # print("B:", B)
    #         i -= 1
    #
    #     return P

    def compute_optimal_dyn_cost_ilqr(self, index, com_opt, mom_opt, F_opt):
        num_eff = len(self.eff_names)
        i = self.horizon-1
        cent_opt = np.concatenate([com_opt[-1], mom_opt[-1]])
        Vx = -2 * np.diag(self.params.W_X_ter) @ cent_opt
        Vxx = np.diag(self.params.W_X_ter)
        # cross_term = np.zeros((12, 9))
        while i >= self.horizon - index:
            # print("i_ilqr:", i)
            A, B = self.linearized_centroidal_dynamics(com_opt[i], mom_opt[i],\
             F_opt[3*num_eff*i : 3*num_eff*(i+1)], i)
            cent_opt = np.concatenate([com_opt[i], mom_opt[i]])
            Qx = -2 * np.diag(self.params.W_X) @ cent_opt + A.T @ Vx
            Qu = -2 * np.diag(self.params.W_F) @ F_opt[3*num_eff*i : 3*num_eff*(i+1)] + B.T @ Vx
            Qxx = np.diag(self.params.W_X) + A.T @ Vxx @ A
            # for j in range(9):
            #     print("b",Bx[j].T @ Vx)
            #     cross_term += Bx[j].T @ Vx
            Qux = B.T @ Vxx @ A
            Quu = np.diag(self.params.W_F) + B.T @ Vxx @ B
            Vx = Qx - Qu @ np.linalg.inv(Quu) @ Qux
            Vxx = Qxx - Qux.T @ np.linalg.inv(Quu) @ Qux
            i -= 1
            # print("cross_term", cross_term)

        return Vx, Vxx

    def optimize(self, q, v, t, v_des, w_des, X_wm = None, F_wm = None, P_wm = None):

        # reseting origin (causes scaling issues I think otherwise)
        q[0:2] = 0
        ## TODO: Needs to be done properly so it is not in the demo file
        if w_des != 0:
            ori_des = q[3:7]
        else:
            ori_des = [0, 0, 0, 1]

        #Move to local frame
        R = pin.Quaternion(np.array(q[3:7])).toRotationMatrix()
        v_des = np.matmul(R, v_des)

        #TODO: Move to C++
        t1 = time.time()
        self.create_cnt_plan(q, v, t, v_des, w_des)
        # print("time:",t)
        #Creates costs for IK and Dynamics
        self.create_costs(q, v, v_des, w_des, ori_des)

        t2 = time.time()

        # pinocchio complains otherwise
        q = pin.normalize(self.rmodel, q)

        self.kd.optimize(q, v, 100, 1)

        t3 = time.time()

        print("Cost Time :", t2 - t1)
        print("Solve Time : ", t3 - t2)
        print(" ================================== ")

        com_opt = self.mp.return_opt_com()
        mom_opt = self.mp.return_opt_mom()
        F_opt = self.mp.return_opt_f()
        xs = self.ik.get_xs()
        us = self.ik.get_us()

        self.time_index = 0 # how many steps from the terminal point
        #ricatti
        # P = self.compute_optimal_dyn_cost_ricatti(self.time_index, com_opt, mom_opt, F_opt)
        # print("P:", np.round(P, 1))
        # self.terminal_cost [int(t/.05)] = P
        # self.terminal_state [int(t/.05)] = np.concatenate([com_opt[-(self.time_index+1)], mom_opt[-(self.time_index+1)]])
        #ilqr
        Vx, Vxx = self.compute_optimal_dyn_cost_ilqr(self.time_index, com_opt, mom_opt, F_opt)
        # print("vxx:", np.round(Vxx, 1))
        self.terminal_hessian [int(t/.05)] = Vxx
        self.terminal_gradient [int(t/.05)] = Vx
        # print("difference:", np.round(Vx-2*P@self.terminal_state[int(t/.05)], 1))



        n_eff = 3*len(self.eff_names)
        for i in range(self.size):
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

        self.q_traj.append(q)
        self.v_traj.append(v)
        self.xs_traj.append(xs)

        return self.xs_int, self.us_int, self.f_int

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

        # Plot Linear Momentum
        fig, ax_m = plt.subplots(3,1)
        ax_m[0].plot(mom_opt[:, 0], label = "Dyn linear_momentum x")
        ax_m[0].plot(ik_mom_opt[:, 0], label="IK linear_momentum x")
        ax_m[1].plot(mom_opt[:, 1], label = "linear_momentum y")
        ax_m[1].plot(ik_mom_opt[:, 1], label="IK linear_momentum y")
        ax_m[2].plot(mom_opt[:, 2], label = "linear_momentum z")
        ax_m[2].plot(ik_mom_opt[:, 2], label="IK linear_momentum z")
        ax_m[0].grid()
        ax_m[0].legend()

        ax_m[1].grid()
        ax_m[1].legend()

        ax_m[2].grid()
        ax_m[2].legend()

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

    def plot_joints(self):
        self.xs_traj = np.array(self.xs_traj)
        self.xs_traj = self.xs_traj[:,:,:self.rmodel.nq]
        self.q_traj = np.array(self.q_traj)
        x = self.dt*np.arange(0, len(self.xs_traj[1]) + len(self.xs_traj), 1)
        # com plots
        fig, ax = plt.subplots(3,1)
        for i in range(len(self.xs_traj)):
            st_hor = i*int(self.planning_time/self.dt)
            ax[0].plot(x[st_hor], self.q_traj[i][10], 'o')
            ax[0].plot(x[st_hor:st_hor + len(self.xs_traj[i])], self.xs_traj[i][:,10])

        plt.show()

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
                                 xs = self.ik.get_xs(),
                                 us = self.ik.get_us(),
                                 cnt_plan = self.cnt_plan)

        print("finished saving ...")
        assert False

    def plot_plan(self, q, v, plot_force = True):
        com_opt = self.mp.return_opt_com()
        mom_opt = self.mp.return_opt_mom()
        F_opt = self.mp.return_opt_f()
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
                ax_f[n].plot(F_opt[3*n::3*self.n_eff], label = self.eff_names[n] + " Fx")
                ax_f[n].plot(F_opt[3*n+1::3*self.n_eff], label = self.eff_names[n] + " Fy")
                ax_f[n].plot(F_opt[3*n+2::3*self.n_eff], label = self.eff_names[n] + " Fz")
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

        plt.show()
