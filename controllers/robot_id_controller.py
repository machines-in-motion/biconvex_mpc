## This file contains an inverse dynamics controller between two frames
## Author : Avadesh Meduri & Paarth Shah
## Date : 16/03/2021

import numpy as np
import pinocchio as pin


class InverseDynamicsController:
    def __init__(self, robot, eff_arr, pinModel = None, pinData = None, real_robot = False):
        """
        Input:
            robot : robot object returned by pinocchio wrapper
            eff_arr : end effector name arr
            real_robot : bool true if controller running on real robot
        """
        if pinModel is None:
            if real_robot is False:
                self.pin_robot = robot

            self.pinModel = self.pin_robot.model
            self.pinData = self.pin_robot.data
            self.nq = self.pin_robot.nq
            self.nv = self.pin_robot.nv

        else:
            self.pinModel = pinModel
            self.pinData = pinData
            self.nq = pinModel.nq
            self.nv = pinModel.nv

        self.wt = 1e-6
        self.mu = 0.8

        self.eff_arr = eff_arr
        self.num_eef = len(eff_arr)

    def set_gains(self, kp, kd):
        """
        This function is used to set the gains
        Input:
            kp : joint proportional gains
            kd : joint derivative gains
        """
        self.kp = kp
        self.kd = kd

    def compute_id_torques(self, q, v, a):
        """
        This function computes the torques for the give state using rnea
        Input:
            q : joint positions
            v : joint velocity
            a : joint acceleration
        """
        return np.reshape(pin.rnea(self.pinModel, self.pinData, q, v, a), (self.nv,))

    def id_joint_torques(self, q, dq, des_q, des_v, des_a, fff, cnt_array):
        """
        This function computes the input torques with gains
        Input:
            q : joint positions
            dq : joint velocity
            des_q : desired joint positions
            des_v : desired joint velocities
            des_a : desired joint accelerations
            fff : desired feed forward force
            cnt_array
        """
        assert len(q) == self.nq
        self.J_arr = []

        # w_com = self.compute_com_wrench(q, dq, des_q, des_v.copy())
        tau_id = self.compute_id_torques(des_q, des_v, des_a)

        for j in range(self.num_eef):
            self.J_arr.append(pin.computeFrameJacobian(self.pinModel, self.pinData, des_q,\
                     self.pinModel.getFrameId(self.eff_arr[j]), pin.LOCAL_WORLD_ALIGNED).T)
            tau_eff = np.zeros(self.nv)
    
        for j in range(self.num_eef):
            if fff[(j*3)+2] > 0:
                tau_eff += np.matmul(self.J_arr[j], np.hstack((fff[j*3:(j+1)*3], np.zeros(3))))

        tau = (tau_id - tau_eff)[6:]

        tau_gain = -self.kp*(np.subtract(q[7:].T, des_q[7:].T)) - self.kd*(np.subtract(dq[6:].T, des_v[6:].T))

        return tau + tau_gain.T
