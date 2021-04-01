# This file contains the solo12 env to simulate the generated trajectories
# Author : Avadesh Meduri
# Date : 11/12/2020

import time
import numpy as np
from matplotlib import pyplot as plt

import pinocchio as pin

from blmc_controllers.robot_impedance_controller import RobotImpedanceController
from blmc_controllers.robot_centroidal_controller import RobotCentroidalController
from blmc_controllers.robot_id_controller import InverseDynamicsController

from bullet_utils.env import BulletEnvWithGround
from raisim_utils.rai_env import RaiEnv

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
        self.bullet = False
        if self.bullet:
            self.env = BulletEnvWithGround()
            self.robot = self.env.add_robot(Solo12Robot)
            self.q0 = np.matrix(Solo12Config.initial_configuration).T
            self.dq0 = np.matrix(Solo12Config.initial_velocity).T
            self.robot.reset_state(self.q0, self.dq0)

        else:
            self.env = RaiEnv()
            urdf_path =  "/home/ameduri/fun_stuff/solo/solo12.urdf"
            self.robot = self.env.add_robot(Solo12Config, urdf_path)
            self.env.launch_server()


        self.rmass = pin.computeTotalMass(self.robot.pin_robot.model)
        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.h_arr = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]

        # Impedance controller gains
        self.kp = self.robot.nb_ee * kp # Disable for now
        self.kd = self.robot.nb_ee * kd
        
        config_file = Solo12Config.paths["imp_ctrl_yaml"]
        self.robot_cent_ctrl = RobotCentroidalController(self.robot,
                mu=0.6, kc=kc, dc=dc, kb=kb, db=db)
        self.robot_leg_ctrl = RobotImpedanceController(self.robot, config_file)

        self.robot_id_ctrl = InverseDynamicsController(self.robot, self.f_arr)
        self.robot_id_ctrl.set_gains(kp[0], kd[0])

        # state estimator
        self.sse = SoloStateEstimator(self.robot.pin_robot)
        # creating arrays
        self.q_des = np.zeros((self.n_col+1, self.robot.pin_robot.nq))
        self.dq_des = np.zeros((self.n_col+1, self.robot.pin_robot.nv))
        self.a_des = np.zeros((self.n_col, self.robot.pin_robot.nv))

        self.x_com = np.zeros((self.n_col+1, 3))
        self.xd_com = np.zeros((self.n_col+1, 3))
        self.x_ori = np.zeros((self.n_col+1, 4))
        self.xd_ori = np.zeros((self.n_col+1, 3))

        self.fff = np.zeros((self.n_col, 12))

        self.x_foot = np.zeros((self.n_col + 1, 12))
        self.xd_foot = np.zeros((self.n_col + 1, 12))

        self.plt_x_foot = np.zeros((self.n_col + 1, 12)) # for ploting

        self.cnt_array = np.zeros((self.n_col, 4))
        self.r_arr_array = np.zeros((self.n_col, 4, 3))

        # creating real data arrays
        self.f_real = np.zeros((self.n_col, 12))
        self.ctrl_f = np.zeros((self.n_col, 12))
        self.ll_real = np.zeros((self.n_col, 12)) # leg length real
        self.foot_real = np.zeros((self.n_col, 12))
        self.x_base_real = np.zeros((self.n_col+1, 3))
        self.x_com_real = np.zeros((self.n_col+1, 3))
        self.xd_com_real = np.zeros((self.n_col+1, 3))


    def generate_motion_plan(self, com_opt, mom_opt, F_opt, cnt_plan, r_arr):
        '''
        This function transforms the output from the solver into arrays that can be tracked
        Input:
            F_opt : optimal force profile
            com_opt : optimal ceneter of mass profile
            mom_opt : optimal momentum profile
            cnt_plan : contact plan
            r_arr : location of contact points throughout the motion
        '''
        for n in range(int(np.round(self.T/self.dt,2))):
            self.x_com[n*self.ratio:(n+1)*self.ratio] = np.linspace(com_opt[n],com_opt[n+1], self.ratio, endpoint = True)
            self.xd_com[n*self.ratio:(n+1)*self.ratio] = np.linspace(mom_opt[n,0:3]/self.rmass,mom_opt[n+1,0:3]/self.rmass,self.ratio)
            self.cnt_array[n*self.ratio:(n+1)*self.ratio] = cnt_plan[n]
            self.r_arr_array[n*self.ratio:(n+1)*self.ratio] = r_arr[n]

            if n < int(np.round(self.T/self.dt,2))-1:
                self.fff[n*self.ratio:(n+1)*self.ratio] = np.linspace(F_opt[12*n:12*(n+1)], F_opt[12*(n+1):12*(n+2)], self.ratio)[:,:,0]
        self.fff[n*self.ratio:] = self.fff[(n)*self.ratio-1]

    def generate_end_eff_plan(self, xs, us):
        '''
        This function creates swing foot trajectories for solo given the plan
        Input:
            xs : joint configuration and velocity from ddp plan
            us : joint torques from ddp plan
        '''
        for n in range(len(xs)):
            q = xs[n][:self.robot.pin_robot.model.nq]
            v = xs[n][self.robot.pin_robot.model.nq:]
            self.q_des[n*self.ratio] = q
            self.dq_des[n*self.ratio] = v
            if n < len(us):
                self.a_des[n*self.ratio] = us[n]
            
            pin.forwardKinematics(self.robot.pin_robot.model, self.robot.pin_robot.data, q, v)
            pin.updateFramePlacements(self.robot.pin_robot.model, self.robot.pin_robot.data) 
            self.x_ori[n*self.ratio] = q[3:7]           
            self.xd_ori[n*self.ratio] = v[3:6]

            for i in range(len(self.f_arr)):
                f_id = self.robot.pin_robot.model.getFrameId(self.f_arr[i])
                h_id = self.robot.pin_robot.model.getFrameId(self.h_arr[i])
                f_loc = self.robot.pin_robot.data.oMf[f_id].translation
                f_vel = pin.getFrameVelocity(self.robot.pin_robot.model, self.robot.pin_robot.data, f_id, pin.LOCAL_WORLD_ALIGNED)
                self.x_foot[n*self.ratio][3*i:3*i+3] = f_loc
                self.xd_foot[n*self.ratio][3*i:3*i+3] = np.array(f_vel)[0:3]
                self.plt_x_foot[n*self.ratio][3*i:3*i+3] = f_loc


        for n in range(len(xs)-1):
            self.x_foot[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.x_foot[n*self.ratio], self.x_foot[(n+1)*self.ratio], self.ratio, endpoint=True)
            self.xd_foot[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.xd_foot[n*self.ratio], self.xd_foot[(n+1)*self.ratio], self.ratio, endpoint=True)

            self.plt_x_foot[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.plt_x_foot[n*self.ratio], self.plt_x_foot[(n+1)*self.ratio], self.ratio, endpoint=True)

            self.x_ori[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.x_ori[n*self.ratio], self.x_ori[(n+1)*self.ratio], self.ratio, endpoint=True)
            self.xd_ori[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.xd_ori[n*self.ratio], self.xd_ori[(n+1)*self.ratio], self.ratio, endpoint=True)

            self.q_des[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.q_des[n*self.ratio], self.q_des[(n+1)*self.ratio], self.ratio, endpoint=True)
            self.dq_des[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.dq_des[n*self.ratio], self.dq_des[(n+1)*self.ratio], self.ratio, endpoint=True)
            if n < len(us) - 1:
                self.a_des[n*self.ratio: (n+1)*self.ratio] = np.linspace(self.a_des[n*self.ratio], self.a_des[(n+1)*self.ratio], self.ratio, endpoint=True)


    def sim(self, fr = 0.0, vname = None):
        '''
        This function simulates the motion plan
        Input: 
            st : step time
            n_steps : number of steps in plan
        '''

        q, dq = self.robot.get_state()
        fl_off, fr_off, hl_off, hr_off = self.sse.return_hip_offset(q, dq)
        self.off = [fl_off, fr_off, hl_off, hr_off]

        if vname:
            self.robot.start_recording(vname)

        for i in range(2000):

            q, dq = self.robot.get_state()
            tau = -10*np.subtract(q[7:], self.q_des[0][7:])
            tau -=  0.5*(dq[6:])
            self.robot.send_joint_command(tau)
            self.env.step() # You can sleep here if you want to slow down the replay


        for i in range(self.n_col):
            time.sleep(fr)
            self.env.step() # You can sleep here if you want to slow down the replay
            
            q, dq = self.robot.get_state()
            self.x_base_real[i] = q[0:3]
            self.x_com_real[i] = pin.centerOfMass(self.robot.pin_robot.model, self.robot.pin_robot.data, q, dq)
            self.xd_com_real[i] = dq[0:3]

            self.f_real[i] = self.robot.get_contact_forces()

            hip_loc = self.sse.return_hip_locations(q, dq)
            hip_vel = self.sse.return_hip_velocities(q, dq)
            foot_loc = self.sse.return_foot_locations(q, dq)            
            self.ll_real[i] = np.reshape(np.subtract(foot_loc, hip_loc), (12,))
            self.foot_real[i] = np.reshape(foot_loc, (12,))

            w_com = self.robot_cent_ctrl.compute_com_wrench(q, dq, self.x_com[i], self.xd_com[i], self.x_ori[i], self.xd_ori[i])
            w_com = np.array(w_com)  
            w_com[0] += np.sum(self.fff[i][0::3]) 
            w_com[1] += np.sum(self.fff[i][1::3]) 
            w_com[2] += np.sum(self.fff[i][2::3]) 

            F = self.robot_cent_ctrl.compute_force_qp(q, dq, self.cnt_array[i], w_com)
            self.ctrl_f[i] = F
            x_des = self.x_foot[i] - np.reshape(hip_loc, (12,))
            xd_des = self.xd_foot[i] - np.reshape(hip_vel, (12,))
            tau = self.robot_leg_ctrl.return_joint_torques(q,dq,self.kp,self.kd, x_des, xd_des,F)
            
            # F = self.fff[i]
            # q_des = self.q_des[i]
            # dq_des = self.dq_des[i]
            # a_des = self.a_des[i]
            # tau = self.robot_id_ctrl.id_joint_torques(q, dq, q_des, dq_des, a_des, F)
            self.robot.send_joint_command(tau)
           
        if vname:
            self.robot.stop_recording()

    def plot(self):

        fig, ax = plt.subplots(3,1)
        ax[0].plot(self.x_com[:,0], label = "Cx")
        ax[0].plot(self.x_com[:,1], label = "Cy")
        ax[0].plot(self.x_com[:,2], label = "Cz")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(self.xd_com[:,0], label = "Vx")
        ax[1].plot(self.xd_com[:,1], label = "Vy")
        ax[1].plot(self.xd_com[:,2], label = "Vz")
        ax[1].grid()
        ax[1].legend()

        ax[2].plot(self.xd_ori[:,0], label = "ang_vel_x")
        ax[2].plot(self.xd_ori[:,1], label = "ang_vel_y")
        ax[2].plot(self.xd_ori[:,2], label = "ang_vel_z")
        ax[2].grid()
        ax[2].legend()

        fig, ax_f = plt.subplots(4,1)
        for n in range(4):
            ax_f[n].plot(self.fff[:,3*n], label = "ee: " + str(n) + "Fx")
            ax_f[n].plot(self.fff[:,3*n+1], label = "ee: " + str(n) + "Fy")
            ax_f[n].plot(self.fff[:,3*n+2], label = "ee: " + str(n) + "Fz")
            ax_f[n].grid()
            ax_f[n].legend()

        fig, ax_foot = plt.subplots(4,1)
        for n in range(4):
            ax_foot[n].plot(self.plt_x_foot[:,3*n], label = "ee: " + str(n) + "foot_x")
            ax_foot[n].plot(self.plt_x_foot[:,3*n+1], label = "ee: " + str(n) + "foot_y")
            ax_foot[n].plot(self.plt_x_foot[:,3*n+2], label = "ee: " + str(n) + "foot_z")
            ax_foot[n].grid()
            ax_foot[n].legend()

        plt.show()


    def plot_real(self):

        fig, ax = plt.subplots(3,1)
        # ax[0].plot(self.x_com[:,0], label = "Cx")
        # ax[0].plot(self.x_com[:,1], label = "Cy")
        ax[0].plot(self.x_com[:,0], label = "Cx")
        ax[0].plot(self.x_com_real[:,0], label = "real_Cx")
        ax[0].plot(self.x_base_real[:,0], label = "real_basex")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(self.x_com[:,1], label = "Cy")
        ax[1].plot(self.x_com_real[:,1], label = "real_Cy")
        ax[1].plot(self.x_base_real[:,1], label = "real_basey")
        ax[1].grid()
        ax[1].legend()
        
        ax[2].plot(self.x_com[:,2], label = "Cz")
        ax[2].plot(self.x_com_real[:,2], label = "real_Cz")
        ax[2].plot(self.x_base_real[:,2], label = "real_basez")
        ax[2].grid()
        ax[2].legend()

        # ax[1].plot(self.ctrl_f[:,0], label = "ctrl_Fx")
        # ax[1].plot(self.ctrl_f[:,1], label = "ctrl_Fy")
        # ax[1].plot(self.fff[:,2], label = "real_Cz")
        # ax[1].grid()
        # ax[1].legend()


        fig, ax_f = plt.subplots(4,1)
        for n in range(4):
            # ax_f[n].plot(self.f_real[:,3*n], label = "ee: " + str(n) + "Fx")
            # ax_f[n].plot(self.f_real[:,3*n+1], label = "ee: " + str(n) + "Fy")
            ax_f[n].plot(self.f_real[:,3*n+2], label = "ee: " + str(n) + "bul_Fz")
            ax_f[n].plot(self.ctrl_f[:,3*n+2], label = "ee: " + str(n) + "ctrl_Fz")
            ax_f[n].plot(self.fff[:,3*n+2], label = "ee: " + str(n) + "plan_Fz")
            ax_f[n].grid()
            ax_f[n].set_ylim(0, 20)
            ax_f[n].legend()

        np.savez("./dat_file/bul", F_real = self.f_real, Com_real = self.x_com_real, \
                                    dCom_real = self.xd_com_real)

        # fig, ax_ll = plt.subplots(4,1)
        # for n in range(4):
        #     ax_ll[n].plot(self.ll_real[:,3*n], label = "ee: " + str(n) + "ll_x")
        #     ax_ll[n].plot(self.ll_real[:,3*n+1], label = "ee: " + str(n) + "ll_y")
        #     ax_ll[n].plot(self.ll_real[:,3*n+2], label = "ee: " + str(n) + "ll_z")
        #     ax_ll[n].grid()
        #     ax_ll[n].legend()

        # fig, ax_foot = plt.subplots(4,1)
        # for n in range(4):
        #     ax_foot[n].plot(self.foot_real[:,3*n], label = "ee: " + str(n) + "foot_x")
        #     ax_foot[n].plot(self.foot_real[:,3*n+1], label = "ee: " + str(n) + "foot_y")
        #     ax_foot[n].plot(self.foot_real[:,3*n+2], label = "ee: " + str(n) + "foot_z")
        #     ax_foot[n].grid()
        #     ax_foot[n].legend()

        plt.show()
