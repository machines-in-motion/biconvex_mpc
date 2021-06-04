 ## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import time
import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP

from matplotlib import pyplot as plt

class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, st, dt, state_wt, x_reg, plan_freq, gait = 1):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            st : duration of step
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
        self.ik = InverseKinematics(r_urdf, dt, st)
        self.st = st
        self.sp = 0.05 # stance phase
        self.dt = dt
        self.plan_freq = plan_freq
        self.foot_size = 0.016/2

        self.col_st = int(np.round(self.st/self.dt,2))
        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.f_id = []
        for i in range(len(self.eff_names)):
            self.f_id.append(self.rmodel.getFrameId(self.eff_names[i]))

        # different gait options
        self.stand = np.array([[1,1,1,1], [1,1,1,1]])
        self.trot = np.array([[1,0,0,1], [0,1,1,0]])
        self.bound = np.array([[1,1,0,0], [0,0,1,1]])
        self.pace = np.array([[1,0,1,0], [0,1,0,1]])

        self.current_contact = np.zeros(4)

        self.wt = [1e4, 1e4]

        if gait == 0:
            self.cnt_gait = self.stand
        elif gait == 1:
            self.cnt_gait = self.trot
        elif gait == 2:
            self.cnt_gait = self.bound
        elif gait == 3:
            self.cnt_gait = self.pace

        self.current_contact = self.cnt_gait[0]

        self.state_wt = state_wt

        self.x_reg = x_reg

        self.m = pin.computeTotalMass(self.rmodel)
        self.rho = 5e+4 # penalty on dynamic constraint violation

        self.mp = BiConvexMP(self.m, self.dt, 2*self.st, len(self.eff_names), rho = self.rho)

        # weights
        self.W_X = np.array([1e-5, 1e-5, 1e+5, 1e+4, 1e+3, 1e+3, 3e3, 3e3, 3e3])

        self.W_X_ter = 100*np.array([1e-5, 1e-5, 1e+5, 1e+4, 1e+4, 1e+4, 1e+5, 1e+5, 1e+5])

        self.W_F = np.array(4*[1e+1, 1e+1, 1e+1])

        # constraints 
        self.bx = 0.25
        self.by = 0.25
        self.bz = 0.25
        self.fx_max = 15
        self.fy_max = 15
        self.fz_max = 15

        self.X_nom = np.zeros((9*int(2*self.st/dt)))

        # for interpolation
        self.xs_int = np.zeros((len(self.x_reg), int(self.dt/0.001)))
        self.us_int = np.zeros((self.rmodel.nv, int(self.dt/0.001)))
        self.f_int = np.zeros((4*len(self.eff_names), int(self.dt/0.001)))

        # plotting
        self.com_traj = []
        self.xs_traj = []

        self.q_traj = []
        self.v_traj = []


    def create_cnt_plan(self, q, v, t, n, next_loc, v_des):

        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        self.cnt_plan = np.zeros((4,len(self.eff_names), 6))
        assert t < self.st

        # first step
        for i in range(len(self.eff_names)):
            self.cnt_plan[0][i][0] = self.cnt_gait[n%2][i]
            #self.cnt_plan[0][i][0] = self.current_contact[i]
            self.cnt_plan[0][i][1:4] = self.rdata.oMf[self.f_id[i]].translation
            self.cnt_plan[0][i][3] += 0.0
            self.cnt_plan[0][i][5] = max(0, self.st - t - self.sp)

        for i in range(len(self.eff_names)):
            self.cnt_plan[1][i][0] = 1.0
            if self.cnt_gait[n%2][i] == 1:
                self.cnt_plan[1][i][1:4] = self.rdata.oMf[self.f_id[i]].translation
            else:
                self.cnt_plan[1][i][1:4] = next_loc[i]
            self.cnt_plan[1][i][3] += self.foot_size
            self.cnt_plan[1][i][4] = max(0, self.st - t - self.sp)    
            self.cnt_plan[1][i][5] = self.st - t    

        for i in range(len(self.eff_names)):
            self.cnt_plan[2][i][0] = self.cnt_gait[(n+1)%2][i]
            self.cnt_plan[2][i][1:4] = next_loc[i]
            self.cnt_plan[2][i][3] += self.foot_size
            self.cnt_plan[2][i][4] = self.st - t    
            self.cnt_plan[2][i][5] = 2*self.st - t

        if t > 0:
            for i in range(len(self.eff_names)):
                self.cnt_plan[3][i][0] = self.cnt_gait[(n+2)%2][i]
                self.cnt_plan[3][i][1:4] = next_loc[i] + v_des*self.st
                self.cnt_plan[2][i][3] += self.foot_size
                self.cnt_plan[3][i][4] = 2*self.st - t    
                self.cnt_plan[3][i][5] = 2*self.st
        else:
            self.cnt_plan = self.cnt_plan[0:3]

        return self.cnt_plan

    def create_costs(self, q, v, v_des, sh, t, wt_xreg, wt_ureg):
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

        # first block
        for i in range(len(self.eff_names)):
            st = self.cnt_plan[0][i][4] 
            et = min(self.cnt_plan[0][i][5], self.st - self.dt)
            if et - st > 0:
                if self.cnt_plan[0][i][0] == 1:
                    N = int(np.round(((et - st)/self.dt),2))
                    pos = np.tile(self.cnt_plan[0][i][1:4], (N,1))
                    self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                    "cnt_" + str(0) + self.eff_names[i])
        
                else:
                    # pos = self.rdata.oMf[i].translation
                    self.ik.add_position_tracking_task(self.f_id[i], et, et, self.cnt_plan[2][i][1:4]\
                                                        , self.wt[1],\
                                                    "end_pos_" + str(0) + self.eff_names[i])
                    
                    if t < (self.st - self.sp)/2.0:
                        pos = self.cnt_plan[2][i][1:4] - v_des*(self.st - self.sp)/2.0
                        pos[2] = sh
                        time = et-(self.st - self.sp)/2.0
                        self.ik.add_position_tracking_task(self.f_id[i],time ,time, pos
                                                        , self.wt[1],\
                                                    "via_" + str(0) + self.eff_names[i])

            # second block
            st = self.cnt_plan[1][i][4] 
            et = min(self.cnt_plan[1][i][5], self.st - self.dt)
            N = int(np.round(((et - st)/self.dt),2))
            if et == self.st - self.dt:
                    pos = self.cnt_plan[1][i][1:4]
                    self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                                                "cnt_" + str(0) + self.eff_names[i])
            else:
                pos = np.tile(self.cnt_plan[1][i][1:4], (N,1))
                self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                "cnt_" + str(0) + self.eff_names[i])

        # third block
        if self.cnt_plan[1][0][5] < self.st: 
            for i in range(len(self.eff_names)):
                st = self.cnt_plan[2][i][4] 
                et = min(self.cnt_plan[2][i][5], self.st)
                if self.cnt_plan[2][i][0] == 1:
                    N = int(np.round(((et - st)/self.dt),2))
                    pos = np.tile(self.cnt_plan[2][i][1:4], (N,1))
                    self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                    "cnt_" + str(1) + self.eff_names[i])
                    self.ik.add_terminal_position_tracking_task(self.f_id[i], pos[0], self.wt[0],\
                                                "cnt_" + str(1) + self.eff_names[i])
                else:
                    if et > st + self.st/2.0:
                        pos = self.cnt_plan[3][i][1:4] - v_des*self.st
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], st + self.st/2.0, st + self.st/2.0, pos
                                                        , self.wt[1],\
                                                    "via_" + str(t) + self.eff_names[i])
                        pos = self.cnt_plan[3][i][1:4]
                        self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                                                "cnt_" + str(1) + self.eff_names[i])
                    else:
                        pos = self.cnt_plan[3][i][1:4] - v_des*self.st/2.0
                        pos[2] = sh
                        self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, 10*self.wt[1],\
                                                "via_" + str(1) + self.eff_names[i])


        self.ik.add_state_regularization_cost(0, self.st, wt_xreg, "xReg", self.state_wt, self.x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, self.st, wt_ureg, "uReg", False)

        self.ik.add_state_regularization_cost(0, self.st, wt_xreg, "xReg", self.state_wt, self.x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, self.st, wt_ureg, "uReg", True)

        self.ik.setup_costs()

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

        amom = 0.5*self.compute_ori_correction(q, np.array([0,0,0,1]))
        self.X_nom[6::9] = amom[0]
        self.X_nom[7::9] = amom[1]
        self.X_nom[8::9] = amom[2]

        X_ter = np.zeros_like(self.X_init)
        X_ter[0:3] = self.X_init[0:3].copy()
        X_ter[2] = 0.2

        X_ter[0:2] = self.X_init[0:2] + (2*v_des*self.st)[0:2]
        X_ter[3:6] = v_des
        X_ter[6:] = amom

        # Setup dynamic optimization

        self.mp.create_contact_array(np.array(self.cnt_plan))
        self.mp.create_bound_constraints(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
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

    def optimize(self, q, v, t, n, next_loc, v_des, sh, x_reg, u_reg, current_contact):
      
        #TODO: Move to C++
        t1 = time.time()
        self.current_contact = current_contact
        self.create_cnt_plan(q, v, t, n, next_loc, v_des)

        #Creates costs for IK and Dynamics
        self.create_costs(q, v, v_des, sh, t, x_reg, u_reg)
        self.q_traj.append(q)
        self.v_traj.append(v)

        # --- Dynamics optimization ---

        t2 = time.time()
        com_opt, F_opt, mom_opt = self.mp.optimize(self.X_init, 65)
        t3 = time.time()

        self.com_traj.append(com_opt)

        # --- IK Optimization ---
        
        # Add tracking costs from Dynamic optimization
        self.ik.add_centroidal_momentum_tracking_task(0, self.st, mom_opt[0:int(self.st/self.dt)], 1e2, "mom_track", False)
        self.ik.add_centroidal_momentum_tracking_task(0, self.st, mom_opt[int(self.st/self.dt)], 1e1, "mom_track", True)

        self.ik.add_com_position_tracking_task(0, self.st, com_opt[0:int(self.st/self.dt)], 1e4, "com_track_cost", False)
        self.ik.add_com_position_tracking_task(0, self.st, com_opt[int(self.st/self.dt)], 1e5, "com_track_cost", True)
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
        self.f_int = np.linspace(F_opt[0*n_eff:n_eff*(1)], F_opt[n_eff*(1):n_eff*(2)], int(self.plan_freq/0.001))
        self.xs_int = np.linspace(xs[0], xs[1], int(self.plan_freq/0.001))
        self.us_int = np.linspace(us[0], us[1], int(self.plan_freq/0.001))
        return self.xs_int, self.us_int, self.f_int

    def reset(self):
        self.ik = InverseKinematics(self.r_urdf, self.dt, self.st)
        self.mp = BiConvexMP(self.m, self.dt, 2*self.st, len(self.eff_names), rho = self.rho)
    

    def plot(self, q_real):
        self.com_traj = np.array(self.com_traj)
        self.q_traj = np.array(self.q_traj)
        self.v_traj = np.array(self.v_traj)
        x = self.dt*np.arange(0, len(self.com_traj[1]) + int((self.plan_freq/self.dt))*len(self.com_traj), 1)
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








                




