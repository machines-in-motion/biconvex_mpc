# This file contains util functions to create contact plans quickly
# It can create contact plans for trot, bound, pace
# Author : Avadesh Meduri
# Date : 11/12/2020


import numpy as np
import pinocchio as pin
import crocoddyl

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator


class SoloCntGen():

    def __init__(self, T, dt, gait = 1):
        '''
        Input:
            T : horizon of plan
            dt : discretization
            gait : 1 = trot, 2 = bound, 3 = pace, 4 = stand
        '''

        robot = Solo12Config.buildRobotWrapper()
        self.rmodel = robot.model
        self.rdata = robot.data 
        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        q0 = np.array(Solo12Config.initial_configuration)

        self.T = T        
        self.foot_size = 0.01 #foot size        

        pin.forwardKinematics(robot.model, robot.data, q0, pin.utils.zero(robot.model.nv))
        pin.updateFramePlacements(robot.model, robot.data)

        self.init_arr =np.array([[[1,0,0, 0.0,0, 0], [1,0,0,0.0,0, 0]\
                                ,[1,0,0,0.0,0,0], [1,0,0,0.0,0,0]]])

        for i in range(len(self.f_arr)):
            f_loc = robot.data.oMf[robot.model.getFrameId(self.f_arr[i])].translation
            self.init_arr[0][i][1:4] = f_loc
            self.init_arr[0][i][3] = 0

        self.gg = GaitGenerator(robot, T, dt)

        if gait == 1:
            self.gait_cnt_arr = [[1,0,0,1], [0,1,1,0]]
        if gait == 2:
            self.gait_cnt_arr = [[1,1,0,0], [0,0,1,1]]
        if gait == 3:
            self.gait_cnt_arr = [[1,0,1,0], [0,1,0,1]]
        if gait == 4:
            self.gait_cnt_arr = [[1,1,1,1], [1,1,1,1]]

    def return_trot_cnt_plan(self, st, sl, i, c_r):
        '''
        Computes contact plan for trot motion for one step
        Input:
            st : step time
            sl : step length (3d)
            i : step number
            c_r : which legs are in contact
        '''
        cnt_plan_n = self.init_arr.copy()
        cnt_plan_n[0,:,4] = i*st
        cnt_plan_n[0,:,5] = (i+1)*st
        cnt_plan_n[0,:,0] = c_r
        for n in range(4):
            if c_r[n] == 1:
                cnt_plan_n[0][n][1:4] += (i-1)*np.array(sl)
            else:
                cnt_plan_n[0][n][1:4] += (i)*np.array(sl)
        
        return cnt_plan_n 

    def create_trot_plan(self, st, sl, n_steps):
        '''
        create an entire contact plan for a trot motion for n steps
        Input:
            st : step time
            sl : step length (3d)
            n : number of step
        '''
        cnt_plan = self.return_trot_cnt_plan(st, [0,0,0], 0, [1,1,1,1])
        for n in range(1,n_steps+1):                
            if  n%2 == 0:
                cnt_plan_n = self.return_trot_cnt_plan(st, sl, n, self.gait_cnt_arr[0])
            else:
                cnt_plan_n = self.return_trot_cnt_plan(st, sl, n, self.gait_cnt_arr[1])

            cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        cnt_plan_n = self.return_trot_cnt_plan(st, sl, n_steps+1, [1,1,1,1])
        cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        return cnt_plan

    def create_ik_step_costs(self, cnt_plan, sh, wt_arr):
        """
        This function creates the IK cost for trot,bound, pace etc.. motion
        Input:
            cnt_plan : output from the dynamic contact plan (above function)
            sh : step height
            wt_arr : weight arr [swing_foot_wt, contact_wt]
        """
        for t in range(0, len(cnt_plan)):
            for i in range(len(self.f_arr)):
                st = np.round(cnt_plan[t][i][4],2) 
                et = np.round(cnt_plan[t][i][5],2)
                e_name = self.f_arr[i]
                if int(cnt_plan[t][i][0]) == 0:
                    f_loc = cnt_plan[t-1][i][1:4].copy()
                    f_loc[2] += self.foot_size 
                    f_loc_next = cnt_plan[t][i][1:4].copy()
                    f_loc_next[2] += self.foot_size 

                    self.gg.create_swing_foot_task(f_loc, \
                        f_loc_next, st, et, sh, e_name, e_name + "_ftc_" + str(t), wt_arr[0])
                
                if int(cnt_plan[t][i][0]) == 1:
                    gr_loc = cnt_plan[t][i][1:4]
                    self.gg.create_contact_task(gr_loc, st, et, e_name, e_name + "_ctc_" + str(t), wt_arr[1])

    def create_contact_costs(self, cnt_plan, wt):
        """
        This function creates costs for end effectors based on contact plan and
        weight array. It can be used for a generic contact plan
        Input:
            cnt_plan : output from the dynamic contact plan (above function)
            wt : contact_wt
        """
        for t in range(0, len(cnt_plan)):
            for i in range(len(self.f_arr)):
                st = np.round(cnt_plan[t][i][4],2) 
                et = np.round(cnt_plan[t][i][5],2)
                e_name = self.f_arr[i]
                if int(cnt_plan[t][i][0]) == 1:
                    gr_loc = cnt_plan[t][i][1:4]
                    gr_loc[2] += self.foot_size
                    self.gg.create_contact_task(gr_loc, st, et, e_name, e_name + "_ctc_" + str(t), wt)

    def create_com_tasks(self, mom_opt, com_opt, wt_arr, ter_wt_arr):
        """
        This function creats center of mass tracking task
        Input:
            mom_opt : optimal momentum trajectory from the motion planner
            com_opt : optimal center of mass trajectory
            wt_arr : [weight for momentum tracking, weight for center of mass tracking]
            ter_wt_arr : terminal [weight for momentum tracking, weight for center of mass tracking]

        """
        self.gg.ik.add_com_position_tracking_task(0, self.T, com_opt, wt_arr[1], "com_track_cost")
        self.gg.create_centroidal_task(mom_opt, 0, self.T, "mom_track_cost", wt_arr[0])

        self.gg.ik.add_com_position_tracking_task(self.T, self.T, \
                                com_opt[-1], ter_wt_arr[1], "com_track_cost", True)
        self.gg.ik.add_centroidal_momentum_tracking_task(self.T, self.T, \
                                mom_opt[-1], ter_wt_arr[0], "mom_track_cost", True)


    def return_gait_generator(self):
        
        return self.gg

    def reset(self, T, dt):

        robot = Solo12Config.buildRobotWrapper()
        self.gg = GaitGenerator(robot, T, dt)
