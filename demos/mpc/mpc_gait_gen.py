## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics


class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, st, dt, , state_wt, x_reg):
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
        self.ik = InverseKinematics(r_urdf, dt, 2*st)
        self.col_st = int(np.round(self.st/self.dt,2))
        self.dt = dt
        self.eff_names = ["FL", "FR", "HL", "HR"]
        self.f_id = []
        for i in range(len(self.eff_names)):
            self.f_id.append(self.rmodel.getFrameId(self.eff_names[i]))
        self.trot = np.array([[1,0,0,1], [0,1,1,0]])

        self.wt = [1e6, 1e4]

        self.cnt_gait =-self.trot

        self.X_init = np.zeros(9)
        self.X_ter = X_init.copy()

        self.state_wt = state_wt

        self.x_reg = x_reg

    def create_costs(self, q, v, v_des, cnt_plan, sh, t):
        """
        Input:
            q : joint positions at current time
            v : joint velocity at current time
            v_des : desired velocity of center of mass
            cnt_plan : contact plan
            sh : step height
            t : time within the step
        """

        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        self.X_init[0:3] = pin.centerOfMass(robot.model, robot.data, q, v) 
        self.X_init[3:6] = q[7:10]

        for k in range(len(cnt_plan)-1):
            for i in range(len(self.eff_names)):
                st = cnt_plan[k][i][4] 
                et = cnt_plan[k][i][5]
                if cnt_plan[t][i][0] == 1:
                    pos = cnt_plan[k][i][1:4]
                    self.ik.add_position_tracking_task(self.f_id[i], st-t, et-t, pos, self.wt[0],\
                                                 "cnt_" + str(t) + self.eff_names[i])

                elif cnt_plan[k][i][0] == 0:
                    pos = cnt_plan[k+1][i][1:4]
                    self.ik.add_position_tracking_task(self.f_id[i], et-t, et-t, pos, self.wt[1],\
                                                 "pos_" + str(t) + self.eff_names[i])
                    if t < self.st/2.0 and k == 0:
                        pos = cnt_plan[k+1][i][1:4] - v_des*st/2.0
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], self.st/2.0 - t, self.st/2.0 - t,\
                                                 pos, self.wt[1],\
                                                 "via_" + str(t) + self.eff_names[i])
                    elif k > 0:
                        pos = 0.5*(cnt_plan[k+1][i][1:4] + cnt_plan[k][i][1:4])
                        pos[2] = sh
                        self.ik.add_position_tracking_task(self.f_id[i], 0.5*(st + et) - t, 0.5*(st + et) - t,\
                                                 pos, self.wt[1],\
                                                 "nxt_via_" + str(t) + self.eff_names[i])
        
        for i in range(len(self.eff_names)):
            st = cnt_plan[-1][i][4] - t
            et = cnt_plan[-1][i][5] - dt - t
            if cnt_plan[t][i][0] == 1:
                pos = cnt_plan[-1][i][1:4]
                self.ik.add_position_tracking_task(self.f_id[i], st-t, et-t, pos, self.wt[0],\
                                                "cnt_" + str(t) + self.eff_names[i])
                self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[0],\
                                                "cnt_" + str(t) + self.eff_names[i])
            elif cnt_plan[-1][i][0] == 0:
                pos = cnt_plan[-2][i][1:4] + v_des*st/2.0
                pos[2] = sh/2.0
                self.ik.add_terminal_position_tracking_task(self.f_id[i], pos, self.wt[1],\
                                                "via_" + str(t) + self.eff_names[i])


        self.ik.add_state_regularization_cost(0, 2*st, wt_xreg, "xReg", state_wt, x_reg, False)
        self.ik.add_ctrl_regularization_cost(0, 2*st, wt_ureg, "uReg", False)

        self.ik.add_state_regularization_cost(0, 2*st, wt_xreg, "xReg", state_wt, x_reg, True)
        self.ik.add_ctrl_regularization_cost(0, 2*st, wt_ureg, "uReg", True)

        self.ik.setup_costs()

    def optimize(self):

        self.ik.optimize(x0) 

        return self.ik.xs






                




