# This file contains the solo12 env to simulate the generated trajectories
# Author : Avadesh Meduri
# Date : 11/12/2020

import numpy as np
from matplotlib import pyplot as plt


from blmc_controllers.robot_impedance_controller import RobotImpedanceController
from blmc_controllers.robot_centroidal_controller import RobotCentroidalController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from . trajectory_generator import TrajGenerator
from .solo_state_estimator import SoloStateEstimator

class Solo12Env:

    def __init__(self, x_init, T, dt, kp, kd, kc, dc, kb, db):
        '''
        Input:
            x_init : initial configuration of solo
            T : duration of motion plan in seconds
            dt : discretization used while generating the plan
            ... gains for the controller (refer to robot com ctrl demo in robot_properties)
        '''
        self.dt = dt
        self.T = T
        self.n_col = np.int(np.round(T/0.001,2))
        self.ratio = int(np.round(self.dt/0.001,2)) # ratio between discretization

        self.env = BulletEnvWithGround()
        self.robot = self.env.add_robot(Solo12Robot)
        initial_configuration = [x_init[0], x_init[1],x_init[2], 0., 0., 0., 1.] + self.robot.nb_ee * [0., 0.9, -1.8]
        Solo12Config.initial_configuration = initial_configuration

        self.q0 = np.matrix(Solo12Config.initial_configuration).T
        dq0 = np.matrix(Solo12Config.initial_velocity).T
        self.robot.reset_state(self.q0, dq0)

        # Impedance controller gains
        self.kp = self.robot.nb_ee * kp # Disable for now
        self.kd = self.robot.nb_ee * kd
        
        config_file = Solo12Config.paths["imp_ctrl_yaml"]
        self.robot_cent_ctrl = RobotCentroidalController(self.robot,
                mu=0.6, kc=kc, dc=dc, kb=kb, db=db)
        self.robot_leg_ctrl = RobotImpedanceController(self.robot, config_file)

        # swing foot trajectory generator
        self.trj_arr = [TrajGenerator(self.robot.pin_robot), TrajGenerator(self.robot.pin_robot), TrajGenerator(self.robot.pin_robot), TrajGenerator(self.robot.pin_robot)]
        self.f_lift = 0.1
        
        # state estimator
        self.sse = SoloStateEstimator(self.robot.pin_robot)
        # creating arrays
        self.x_com = np.zeros((self.n_col, 3))
        self.xd_com = np.zeros((self.n_col, 3))
        self.x_ori = np.zeros((self.n_col, 4))
        self.x_ori[:,3] = 1
        self.xd_ori = np.zeros((self.n_col, 3))

        self.cnt_array = np.zeros((self.n_col, 4))
        self.r_arr_array = np.zeros((self.n_col, 4, 3))

    def motion_plan(self, X_opt, F_opt, cnt_plan, r_arr):
        '''
        This function transforms the output from the solver into arrays that can be tracked
        Input:
            F_opt : optimal force profile
            X_opt : optimal ceneter of mass profile
            cnt_plan : contact plan
            r_arr : location of contact points throughout the motion
        '''
        for n in range(int(np.round(self.T/self.dt,2))):
            for j in range(3):
                self.x_com[n*self.ratio:(n+1)*self.ratio,j] = np.linspace(X_opt[9*n+j,0],X_opt[9*n+9+j,0], self.ratio, endpoint = True)
                self.xd_com[n*self.ratio:(n+1)*self.ratio,j] = np.linspace(X_opt[9*n+j+3,0],X_opt[9*n+12+j,0],self.ratio)
                self.xd_ori[n*self.ratio:(n+1)*self.ratio,j] = np.linspace(X_opt[9*n+j+6,0],X_opt[9*n+15+j,0],self.ratio)
            
            self.cnt_array[n*self.ratio:(n+1)*self.ratio] = cnt_plan[n]
            self.r_arr_array[n*self.ratio:(n+1)*self.ratio] = r_arr[n]
        
        self.xd_ori = np.divide(self.xd_ori, np.diag(self.robot.pin_robot.mass(self.q0)[3:6, 3:6]))

    def generate_end_eff_plan(self, q, dq, foot_loc, foot_arr, cnt_plan, x_com, t, step_time):
        '''
        This function creates swing foot trajectories for solo given the plan
        Input:
            q, dq : joint configuration and velocity
            foot_loc : foot location
            foot_arr : contains 4*3 array of desired next foot locations
            cnt_plan : contains the contact plan of next foot location
            x_com : current desired com location
            t : duration within the step
            step_time : duration of step time
        '''

        x_des = 4*[0.0, 0.0, 0.0]
        xd_des = 4*[0.0, 0.0, 0]
        
        hip_loc = self.off.copy() + np.array([0,0,0.2])
        for n in range(4):
            foot_loc[n][2] = 0.0
            if cnt_plan[n] == 0.0:
                x_des[n*3:n*3+3], xd_des[n*3:n*3+3] = self.trj_arr[n].generate_foot_traj(foot_loc[n].copy(), \
                                       foot_arr[n] , [0.0, 0.0, self.f_lift], step_time,t)
            elif cnt_plan[n] == 1.0:
                x_des[n*3:n*3+3], xd_des[n*3:n*3+3] = self.trj_arr[n].generate_foot_traj(foot_loc[n].copy(), \
                                       foot_loc[n].copy() , [0.0, 0.0, 0.0], step_time,t)
            
            x_des[n*3:n*3+3] = np.subtract(x_des[n*3:n*3+3],hip_loc[n])
        return x_des, xd_des

    def sim(self, st, n_steps):
        '''
        This function simulates the motion plan
        Input: 
            st : step time
            n_steps : number of steps in plan
        '''
        step_time = int(st/0.001)
        k = 0
        q, dq = self.robot.get_state()
        fl_off, fr_off, hl_off, hr_off = self.sse.return_hip_offset(q, dq)
        self.off = [fl_off, fr_off, hl_off, hr_off]

        for i in range(self.n_col):
            self.env.step(sleep=True) # You can sleep here if you want to slow down the replay
            q, dq = self.robot.get_state()
            w_com = self.robot_cent_ctrl.compute_com_wrench(q, dq, self.x_com[i], self.xd_com[i], self.x_ori[i], self.xd_ori[i])
            tmp = (int(i//step_time))
            if k == 0:
                fl_foot, fr_foot, hl_foot, hr_foot = self.sse.return_foot_locations(q, dq) ## computing current location of the feet
                foot_loc = [fl_foot, fr_foot, hl_foot, hr_foot]

            # F = self.robot_cent_ctrl.compute_force_qp(q, dq, self.cnt_array[tmp*step_time], w_com)
            x_des_tmp, xd_des_tmp = self.generate_end_eff_plan(q, dq, foot_loc, self.r_arr_array[tmp*step_time], \
                                self.cnt_array[tmp*step_time], self.x_com[i], 0.001*k, st)
            F = np.zeros(12)
            x_des = 12*[0,]
            x_des[2] = x_des_tmp[2]
            x_des[5] = -0.25
            x_des[8] = x_des_tmp[8]
            x_des[11] = -0.25
            xd_des = 12*[0,]
            tau = self.robot_leg_ctrl.return_joint_torques(q,dq,self.kp,self.kd, x_des, xd_des,F)

            self.robot.send_joint_command(tau)
            k += 1
            k = k%step_time
