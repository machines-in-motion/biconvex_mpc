# This file contains util functions to create contact plans quickly
# Author : Avadesh Meduri
# Date : 11/12/2020


import numpy as np
import pinocchio as pin
import crocoddyl

from robot_properties_solo.config import Solo12Config
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator


class SoloCntGen:

    def __init__(self, T, dt):
        '''
        Input:
            T : horizon of plan
            dt : discretization
        '''

        robot = Solo12Config.buildRobotWrapper()
        self.rmodel = robot.model
        self.rdata = robot.data 
        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        q0 = np.array(Solo12Config.initial_configuration)
        
        pin.forwardKinematics(robot.model, robot.data, q0, pin.utils.zero(robot.model.nv))
        pin.updateFramePlacements(robot.model, robot.data)

        self.init_arr =np.array([[[1,0,0, 0.0,0, 0], [1,0,0,0.0,0, 0]\
                                ,[1,0,0,0.0,0,0], [1,0,0,0.0,0,0]]])

        for i in range(len(self.f_arr)):
            f_loc = robot.data.oMf[robot.model.getFrameId(self.f_arr[i])].translation
            self.init_arr[0][i][1:4] = f_loc
        
        self.gg = GaitGenerator(robot, T, dt)

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
        cnt_plan_n[0,:,1:4] += sl
        
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
                cnt_plan_n = self.return_trot_cnt_plan(st, n*sl, n, [1,0,1,0])
            else:
                cnt_plan_n = self.return_trot_cnt_plan(st, n*sl, n, [0,1,0,1])
            cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        cnt_plan_n = self.return_trot_cnt_plan(st, (n_steps)*sl, n_steps+1, [1,1,1,1])
        cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        return cnt_plan

    def create_ik_step_costs(self, cnt_plan):

        for t in range(len(cnt_plan)-1):
            for i in range(len(self.f_arr)):
                if cnt_plan[t][i][0] == 0:
                    f_loc = cnt_plan[t][i][1:4]
                    f_loc_next = cnt_plan[t+1][i][1:4]
                    gg.create_swing_foot_task(f_loc, fl_loc_next, 0.0, 0.5, 0.1, "FL_FOOT", "FL_ftc1", 1e2)

        gg.create_contact_task(fr_loc, 0.0, 0.5, "FR_FOOT", "FR_ctc1", 1e3)
        gg.create_contact_task(hl_loc, 0.0, 0.5, "HL_FOOT", "HL_ctc1", 1e3)