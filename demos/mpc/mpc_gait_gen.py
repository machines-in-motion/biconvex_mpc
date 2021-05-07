## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP

from matplotlib import pyplot as plt

class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, st, dt, state_wt, x_reg):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            st : duration of step
            dt : discretization
            state_wt : state regularization wt
            x_reg : joint config about which regulation is done
        """     

        ## Note : only creates plan for horizon of 2*st
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        self.ik = InverseKinematics(r_urdf, dt, st)
        self.st = st
        self.dt = dt

        self.col_st = int(np.round(self.st/self.dt,2))
        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.f_id = []
        for i in range(len(self.eff_names)):
            self.f_id.append(self.rmodel.getFrameId(self.eff_names[i]))
        self.trot = np.array([[1,0,0,1], [0,1,1,0]])

        self.wt = [1e6, 1e4]

        self.cnt_gait = self.trot
        self.state_wt = state_wt

        self.x_reg = x_reg

        self.m = pin.computeTotalMass(self.rmodel)
        self.rho = 5e+4 # penalty on dynamic constraint violation

        self.mp = BiConvexMP(self.m, self.dt, 2*self.st, len(self.eff_names), rho = self.rho)

        # weights
        self.W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 3e3, 3e3, 3e3])

        self.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5, 1e+5])

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

    def create_cnt_plan(self, q, v, t, n, next_loc, v_des):

        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        self.cnt_plan = np.zeros((3,len(self.eff_names), 6))
        assert t < self.st
        # first step
        for i in range(len(self.eff_names)):
            self.cnt_plan[0][i][0] = self.cnt_gait[n%2][i]
            self.cnt_plan[0][i][1:4] = self.rdata.oMf[self.f_id[i]].translation
            self.cnt_plan[0][i][5] = self.st - t    

        for i in range(len(self.eff_names)):
            self.cnt_plan[1][i][0] = self.cnt_gait[(n+1)%2][i]
            self.cnt_plan[1][i][1:4] = next_loc[i]
            self.cnt_plan[1][i][4] = self.st - t    
            self.cnt_plan[1][i][5] = 2*self.st - t

        if t > 0:
            for i in range(len(self.eff_names)):
                self.cnt_plan[2][i][0] = self.cnt_gait[(n+2)%2][i]
                self.cnt_plan[2][i][1:4] = next_loc[i] + v_des*self.st
                self.cnt_plan[2][i][4] = 2*self.st - t    
                self.cnt_plan[2][i][5] = 2*self.st
        else:
            self.cnt_plan = self.cnt_plan[0:2]

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
            et = min(self.cnt_plan[1][i][5], self.st - self.dt)
            if self.cnt_plan[0][i][0] == 1:
                N = int(np.round(((et - st)/self.dt),2))
                pos = np.tile(self.cnt_plan[0][i][1:4], (N,1))
                self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                 "cnt_" + str(0) + self.eff_names[i])
                if et == self.st - self.dt:
                    self.ik.add_terminal_position_tracking_task(self.f_id[i], pos[0], self.wt[0],\
                                                "cnt_" + str(0) + self.eff_names[i])

            else:
                # pos = self.rdata.oMf[i].translation
                self.ik.add_position_tracking_task(self.f_id[i], et, et, self.cnt_plan[1][i][1:4]\
                                                    , self.wt[1],\
                                                 "end_pos_" + str(0) + self.eff_names[i])
                
                if t < self.st/2.0:
                    pos = self.cnt_plan[1][i][1:4] - v_des*self.st/2.0
                    pos[2] = sh
                    self.ik.add_position_tracking_task(self.f_id[i],et-self.st/2.0,et-self.st/2.0, pos
                                                    , self.wt[1],\
                                                 "via_" + str(0) + self.eff_names[i])

        # #second block
        if self.cnt_plan[0][0][5] < self.st: 
            for i in range(len(self.eff_names)):
                st = self.cnt_plan[1][i][4] 
                et = min(self.cnt_plan[1][i][5], self.st)
                if self.cnt_plan[1][i][0] == 1:
                    N = int(np.round(((et - st)/self.dt),2))
                    pos = np.tile(self.cnt_plan[1][i][1:4], (N,1))
                    self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                    "cnt_" + str(1) + self.eff_names[i])
                    self.ik.add_terminal_position_tracking_task(self.f_id[i], pos[0], self.wt[0],\
                                                "cnt_" + str(1) + self.eff_names[i])

                else:
                    if et > st + self.st/2.0:
                        pos = self.cnt_plan[2][i][1:4] - v_des*self.st/2.0
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], st + self.st/2.0, st + self.st/2.0, pos
                                                        , self.wt[1],\
                                                    "via_" + str(t) + self.eff_names[i])
                        pos = self.cnt_plan[2][i][1:4]
                        self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                                                "cnt_" + str(1) + self.eff_names[i])
                    else:
                        pos = self.cnt_plan[2][i][1:4] - v_des*self.st/2.0
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
        self.X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
        
        X_ter = self.X_init.copy()
        X_ter[0:3] += v_des*self.st*2
        X_ter[3:6] = v_des*self.m

        self.mp.create_contact_array(np.array(self.cnt_plan))
        self.mp.create_bound_constraints(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(self.W_X, self.W_X_ter, X_ter, self.X_nom)
        self.mp.create_cost_F(self.W_F)


    def optimize(self, q, v, t, n, next_loc, v_des, sh, x_reg, u_reg):
      
        self.create_cnt_plan(q, v, t, n, next_loc, v_des)
        self.create_costs(q, v, v_des, sh, t, x_reg, u_reg)

        com_opt, F_opt, mom_opt = self.mp.optimize(self.X_init, 50)
        self.ik.add_centroidal_momentum_tracking_task(0, self.st, mom_opt[0:int(self.st/self.dt)], 1e3, "mom_track", False)
        self.ik.add_centroidal_momentum_tracking_task(0, self.st, mom_opt[int(self.st/self.dt)], 1e3, "mom_track", True)

        self.ik.add_com_position_tracking_task(0, self.st, com_opt[0:int(self.st/self.dt)], 1e2, "com_track_cost", False)
        self.ik.add_com_position_tracking_task(0, self.st, com_opt[int(self.st/self.dt)], 1e2, "com_track_cost", True)

        self.ik.optimize(self.x0) 
        xs = self.ik.get_xs()
        us = self.ik.get_us()
        n_eff = 3*len(self.eff_names)
        self.f_int = np.linspace(F_opt[n_eff:n_eff*(2)], F_opt[n_eff*(2):n_eff*(3)], len(self.f_int))
        self.xs_int = np.linspace(xs[1], xs[2], len(self.xs_int))
        self.us_int = np.linspace(us[1], us[2], len(self.us_int))

        return self.xs_int, self.us_int, self.f_int

    def reset(self):
        self.ik = InverseKinematics(self.r_urdf, self.dt, self.st)
        self.mp = BiConvexMP(self.m, self.dt, 2*self.st, len(self.eff_names), rho = self.rho)
    
    def plot(self):
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








                




