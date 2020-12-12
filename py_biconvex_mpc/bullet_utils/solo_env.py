# This file contains the solo12 env to simulate the generated trajectories
# Author : Avadesh Meduri
# Date : 11/12/2020

import numpy as np

from blmc_controllers.robot_impedance_controller import RobotImpedanceController
from blmc_controllers.robot_centroidal_controller import RobotCentroidalController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

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

        # creating arrays
        self.x_com = np.zeros((self.n_col, 3))
        self.xd_com = np.zeros((self.n_col, 3))
        self.x_ori = np.zeros((self.n_col, 4))
        self.x_ori[:,3] = 1
        self.xd_ori = np.zeros((self.n_col, 3))

    def motion_plan(self, X_opt, F_opt):
        '''
        This function transforms the output from the solver into arrays that can be tracked
        Input:
            F_opt : optimal force profile
            X_opt : optimal ceneter of mass profile
            T : duration of motion
            dt : discretization used in the plan
        '''
        for n in range(int(np.round(self.T/self.dt,2))):
            for j in range(3):
                self.x_com[n*self.ratio:(n+1)*self.ratio,j] = np.linespace(X_opt[9*n+j],X_opt[9*n+9+j],self.ratio)
                self.xd_com[n*self.ratio:(n+1)*self.ratio,j] = np.linespace(X_opt[9*n+j+3],X_opt[9*n+12+j],self.ratio)
                self.xd_ori[n*self.ratio:(n+1)*self.ratio,j] = np.linespace(X_opt[9*n+j+6],X_opt[9*n+15+j],self.ratio)
            
        self.xd_ori = np.divide(self.xd_ori, np.diag(self.robot.pin_robot.mass(self.q0)[3:6, 3:6]))

    def simulate(self):

        for i in range(self.n_col):
            self.env.step(sleep=True) # You can sleep here if you want to slow down the replay
            q, dq = self.robot.get_state()
            w_com = self.robot_cent_ctrl.compute_com_wrench(q, dq, self.x_com[i], self.xd_com[i], self.x_ori[i], self.x_angvel[i])
            F = self.robot_cent_ctrl.compute_force_qp(q, dq, self.cnt_array[i], w_com)
            tau = self.robot_leg_ctrl.return_joint_torques(q,dq,self.kp,self.kd, self.x_des[i], self.xd_des[i],F)
            self.robot.send_joint_command(tau)