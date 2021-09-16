## This file contains an inverse dynamics controller between two frames
## Author : Avadesh Meduri
## Date : 16/03/2021

import numpy as np
import pinocchio as pin
import time

#from . qp_solver import quadprog_solve_qp

arr = lambda a: np.array(a).reshape(-1)
mat = lambda a: np.matrix(a).reshape((-1, 1))

class InverseDynamicsController():

    def __init__(self, robot, eff_arr, pinModel = None, pinData = None, real_robot = False):
        """
        Input:
            robot : robot object returned by pinocchio wrapper
            eff_arr : end effector name arr
            real_robot : bool true if controller running on real robot
        """

        # if real_robot:
        #     self.pin_robot = robot
        # else:
        #     self.pin_robot = robot.pin_robot

        if pinModel == None:
            if real_robot:
                self.pin_robot = robot
            else:
                self.pin_robot = robot.pin_robot

            self.pinModel = self.pin_robot.model
            self.pinData = self.pin_robot.data
            self.nq = self.pin_robot.nq
            self.nv = self.pin_robot.nv

        else:
            self.pinModel = pinModel
            self.pinData = pinData
            self.nq = pinModel.nq
            self.nv = pinModel.nv

        self.robot_mass = pin.computeTotalMass(self.pinModel)
        self.eff_arr = eff_arr

        self.wt = 1e-6
        self.mu = 0.8

    def set_gains(self, kp, kd, kc, dc, kb, db):
        """
        This function is used to set the gains
        Input:
            kp : joint proportional gains
            kd : joint derivative gains
        """
        self.kp = kp
        self.kd = kd

        assert len(kc) == 3
        assert len(dc) == 3
        assert len(kb) == 3
        assert len(db) == 3

        self.kc = kc
        self.dc = dc
        self.kb = kb
        self.db = db

    def compute_id_torques(self, q, v, a):
        """
        This function computes the torques for the give state using rnea
        Input:
            q : joint positions
            v : joint velocity
            a : joint acceleration
        """
        return np.reshape(pin.rnea(self.pinModel, self.pinData, q, v, a), (self.nv,))

    def compute_com_wrench(self, q, v, des_q, des_v):
        """
        This function computes the centroidal error wrench that is used in the QP
        Input:
            q : joint positions
            v : joint velocity
            des_q : desired joint positions
            des_v : desired joint velocities
        """
        des_com = pin.centerOfMass(self.pinModel, self.pinData, des_q, des_v)
        com = pin.centerOfMass(self.pinModel, self.pinData, q, v)
        vcom = np.reshape(np.array(v[0:3]), (3,))
        Ib = self.pin_robot.mass(q)[3:6, 3:6]

        cur_angvel = arr(v[3:6])
        des_angvel = arr(des_v[3:6])
        # des_angvel = np.zeros(3)

        # quat_diff = self.quaternion_difference(arr(q[3:7]), arr(np.array([0,0,0,1])))
        quat_diff = self.quaternion_difference(arr(q[3:7]), arr(des_q[3:7]))
        
        w_com = np.hstack([
            np.multiply(self.kc, des_com - com) + self.robot_mass * np.multiply(self.dc, arr(des_v[0:3]) - vcom),
            arr(arr( np.multiply(self.kb, quat_diff)) + arr((Ib * mat(np.multiply(self.db, des_angvel - cur_angvel))).T))
        ])

        return w_com

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

        ## creating QP matrices
        N = int(len(self.eff_arr))

        # Q = self.wt*np.eye(3*N + 6)
        # Q[-6:-3, -6:-3] *= 1e9
        # p = np.zeros(3*N + 6)
        # p[:3*N] = -2*self.wt*np.array(fff)

        # G = np.zeros((5 * N, 3 * N + 6))
        # h = np.zeros((5 * N ))

        # A = np.zeros((6, 3*N + 6))
        for j in range(N):
            self.J_arr.append(pin.computeFrameJacobian(self.pinModel, self.pinData, des_q,\
                     self.pinModel.getFrameId(self.eff_arr[j]), pin.LOCAL_WORLD_ALIGNED).T)
        #     # if cnt_array[j] == 0:
        #     if fff[3*j+2] < 0.01:
        #         continue
        #     A[:,3*j: 3*(j+1)] = self.J_arr[j].copy()[0:6,0:3]
            
        #     G[5*j + 0, 3 * j + 0] = 1    # mu Fz - Fx >= 0
        #     G[5*j + 0, 3 * j + 2] = -self.mu
        #     G[5*j + 1, 3 * j + 0] = -1     # mu Fz + Fx >= 0
        #     G[5*j + 1, 3 * j + 2] = -self.mu
        #     G[5*j + 2, 3 * j + 1] = 1    # mu Fz - Fy >= 0
        #     G[5*j + 2, 3 * j + 2] = -self.mu
        #     G[5*j + 3, 3 * j + 1] = -1     # mu Fz + Fy >= 0
        #     G[5*j + 3, 3 * j + 2] = -self.mu
        #     G[5*j + 4, 3 * j + 2] = -1     # Fz >= 0
        # A[:, -6:] = np.eye(6)

        # b = tau_id[0:6] + w_com
        # print(tau_id[2], fff[2::3])

        # solx = quadprog_solve_qp(Q, p, G, h, A, b)
        # qp_fff = solx[:3*N]
        # print(tau_id[2], fff[2::3], qp_fff[2::3])
        # print(solx[3*N:])

        tau_eff = np.zeros(self.nv)
        for j in range(N):
            if fff[(j*3)+2] > 0:
                tau_eff += np.matmul(self.J_arr[j], np.hstack((fff[j*3:(j+1)*3], np.zeros(3))))

        tau = (tau_id - tau_eff)[6:]

        tau_gain = -self.kp*(np.subtract(q[7:].T, des_q[7:].T)) - self.kd*(np.subtract(dq[6:].T, des_v[6:].T))

        return tau + tau_gain.T


     #### quaternion stuff
    def skew(self, v):
        '''converts vector v to skew symmetric matrix'''
        assert v.shape[0] == 3, 'vector dimension is not 3 in skew method'
        return np.array([[0., -v[2], v[1]],
                         [v[2], 0., -v[0]],
                         [-v[1], v[0], 0.]])
    def quaternion_to_rotation(self, q):
        ''' converts quaternion to rotation matrix '''
        return (q[3]**2 - q[:3].dot(q[:3]))*np.eye(3) \
            + 2. * np.outer(q[:3], q[:3]) + 2.*q[3]*self.skew(q[:3])

    def exp_quaternion(self, w):
        ''' converts angular velocity to quaternion '''
        qexp = np.zeros(4)
        th = np.linalg.norm(w)
        if th**2 <= 1.e-6:
            ''' small norm causes closed form to diverge,
            use taylor expansion to approximate '''
            qexp[:3] = (1-(th**2)/6)*w
            qexp[3] = 1-(th**2)/2
        else:
            u = w/th
            qexp[:3] = np.sin(th)*u
            qexp[3] = np.cos(th)
        return qexp

    def log_quaternion(self, q):
        """ lives on the tangent space of SO(3) """
        v = q[:3]
        w = q[3]
        vnorm = np.linalg.norm(v)
        if vnorm <= 1.e-6:
            return 2 * v/w * (1 - vnorm**2/(3*w**2))
        else:
            return 2*np.arctan2(vnorm, w) * v / vnorm

    def quaternion_product(self, q1, q2):
        """ computes quaternion product of q1 x q2 """
        p = np.zeros(4)
        p[:3] = np.cross(q1[:3], q2[:3]) + q2[3]*q1[:3] + q1[3]*q2[:3]
        p[3] = q1[3]*q2[3] - q1[:3].dot(q2[:3])
        return p

    def integrate_quaternion(self, q, w):
        """ updates quaternion with tangent vector w """
        dq = self.exp_quaternion(.5*w)
        return self.quaternion_product(dq, q)

    def quaternion_difference(self, q1, q2):
        """computes the tangent vector from q1 to q2 at Identity
        returns vecotr w
        s.t. q2 = exp(.5 * w)*q1
        """
        # first compute dq s.t.  q2 = q1*dq
        q1conjugate = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
        # order of multiplication is very essential here
        dq = self.quaternion_product(q2, q1conjugate)
        # increment is log of dq
        return self.log_quaternion(dq)
