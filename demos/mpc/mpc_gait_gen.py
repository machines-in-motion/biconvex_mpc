## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import numpy as np
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics


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

        # self.X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q, v) 
        # self.X_init[3:6] = q[7:10]

        # first block
        for i in range(len(self.eff_names)):
            st = self.cnt_plan[0][i][4] 
            et = self.cnt_plan[0][i][5]
            if self.cnt_plan[0][i][0] == 1:
                N = int(np.round(((et - st)/self.dt),2))
                pos = np.tile(self.cnt_plan[0][i][1:4], (N,1))
                self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                 "cnt_" + str(t) + self.eff_names[i])

            else:
                pos = self.rdata.oMf[i].translation
                self.ik.add_position_tracking_task(self.f_id[i], st, st, pos, self.wt[1],\
                                                 "pos_" + str(t) + self.eff_names[i])

                self.ik.add_position_tracking_task(self.f_id[i], et-self.dt, et-self.dt, self.cnt_plan[1][i][1:4]\
                                                    , self.wt[1],\
                                                 "end_pos_" + str(t) + self.eff_names[i])
                
                if t < self.st/2.0:
                    pos = self.cnt_plan[1][i][1:4] - v_des*self.st/2.0
                    pos[2] = sh
                    self.ik.add_position_tracking_task(self.f_id[i], 0.5*(st+et), 0.5*(st+et), pos
                                                    , 0.1*self.wt[1],\
                                                 "via_" + str(t) + self.eff_names[i])

        #second block
        if self.cnt_plan[0][0][5] < self.st - self.dt: 
            for i in range(len(self.eff_names)):
                st = self.cnt_plan[1][i][4] 
                et = min(self.cnt_plan[1][i][5], self.st)
                print(et)
                if self.cnt_plan[1][i][0] == 1:
                    N = int(np.round(((et - st)/self.dt),2))
                    pos = np.tile(self.cnt_plan[0][i][1:4], (N,1))
                    self.ik.add_position_tracking_task(self.f_id[i], st, et, pos, self.wt[0],\
                                                    "cnt_" + str(t) + self.eff_names[i])
                    # self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                    #                             "cnt_" + str(t) + self.eff_names[i])

                else:
                    if et > st + self.st/2.0:
                        pos = self.cnt_plan[2][i][1:4] - v_des*self.st/2.0
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], st + self.st/2.0, st + self.st/2.0, pos
                                                        , 0.1*self.wt[1],\
                                                    "via_" + str(t) + self.eff_names[i])
                        pos = self.cnt_plan[2][i][1:4]
                        # self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                        #                         "cnt_" + str(t) + self.eff_names[i])
                    else:
                        pos = self.cnt_plan[2][i][1:4] - v_des*self.st/2.0
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], et-self.dt, et-self.dt, pos
                                                        , 0.01*self.wt[1],\
                                                    "via_" + str(t) + self.eff_names[i])
                        # self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[1],\
                        #                         "cnt_" + str(t) + self.eff_names[i])

        # t = len(self.cnt_plan) - 1
        # st = self.cnt_plan[t][i][4] 
        # et = min(self.cnt_plan[t][i][5], self.st)
        # for i in range(len(self.eff_names)):
        #     if self.cnt_plan[t][i][0] == 1:
        #         pos = self.cnt_plan[t][i][1:4]
        #         self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
        #                                         "cnt_" + str(t) + self.eff_names[i])
        #     elif self.cnt_plan[t][i][0] == 0:
        #         if et - st < self.st/2.0:
        #                 pos = self.cnt_plan[t][i][1:4] - v_des*(et - st)
        #                 pos[2] = sh
        #         else:
        #             pos = self.cnt_plan[t][i][1:4]

        #         self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[1],\
        #                                         "via_" + str(t) + self.eff_names[i])


        self.ik.add_state_regularization_cost(0, st, wt_xreg, "xReg", self.state_wt, self.x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, st, wt_ureg, "uReg", False)

        self.ik.add_state_regularization_cost(0, st, wt_xreg, "xReg", self.state_wt, self.x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, st, wt_ureg, "uReg", True)

        self.ik.setup_costs()

    def optimize(self, x0):

        self.ik.optimize(x0) 

        return self.ik.get_xs()






                




