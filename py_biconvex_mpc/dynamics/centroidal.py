# This file contains the implementation of the centroidal dynamics
# it computes analytical derivatives of the dynamics
# Author : Avadesh Meduri
# Date : 5/12/2020

import numpy as np

class CentroidalDynamics:

    def __init__(self, m, dt, T, n_eff):
        '''
        Input:
            m : mass of the robot
            dt : discretization of the dynamics
            n_eff : number of end effectors
            T : horizon of plan 
        '''
        self.g = 9.81
        self.m = m
        self.dt = dt
        self.n_col = int(np.round(T/self.dt,2))
        self.n_eff = n_eff

        self.block_A_f = np.zeros((9,18))
        self.block_A_f[0:9,0:9] = np.identity(9)
        self.block_A_f[0:9,9:18] = -np.identity(9)
        self.block_A_f[0:3,3:6] = self.dt*np.identity(3)

        self.block_b_f = np.zeros(9)

        self.block_A_x = np.zeros((9,3*self.n_eff))            
        
        self.block_b_x = np.zeros(9)

    def create_block_f(self, F_t, r_t, cnt):
        '''
        Creates the block A(F)*x -b(F) = 0 for the given time step
        Input:
            F_t : force vector for current time step (3*n)
            r_t : location of the contact points
            cnt : contact array (1 if in contact, 0 otherwise)
        '''
        self.block_A_f[6:9,0:3] = np.zeros((3,3))
        self.block_b_f[3:] = 0

        for ee in range(self.n_eff):
            self.block_A_f[6,1] -= cnt[ee]*F_t[3*ee +2]*self.dt
            self.block_A_f[6,2] += cnt[ee]*F_t[3*ee +1]*self.dt

            self.block_A_f[7,0] += cnt[ee]*F_t[3*ee+2]*self.dt
            self.block_A_f[7,2] -= cnt[ee]*F_t[3*ee+0]*self.dt

            self.block_A_f[8,0] -= cnt[ee]*F_t[3*ee+1]*self.dt
            self.block_A_f[8,1] += cnt[ee]*F_t[3*ee+0]*self.dt
            
            self.block_b_f[3] -= cnt[ee]*F_t[3*ee+0]*self.dt/self.m
            self.block_b_f[4] -= cnt[ee]*F_t[3*ee+1]*self.dt/self.m
            self.block_b_f[5] -= cnt[ee]*F_t[3*ee+2]*self.dt/self.m 
            
            self.block_b_f[6] += (cnt[ee]*F_t[3*ee+1]*r_t[ee,2] - cnt[ee]*F_t[3*ee+2]*r_t[ee,1])*self.dt
            self.block_b_f[7] += (cnt[ee]*F_t[3*ee+2]*r_t[ee,0] - cnt[ee]*F_t[3*ee+0]*r_t[ee,2])*self.dt
            self.block_b_f[8] += (cnt[ee]*F_t[3*ee+0]*r_t[ee,1] - cnt[ee]*F_t[3*ee+1]*r_t[ee,0])*self.dt

        self.block_b_f[5] += self.g*self.dt

    def create_block_x(self, X_t, r_t, cnt):
        '''
        Creates the block A(X)*F - b(X) = 0 for the given time step
        Input:
            X_t : state vector at current time step (18)
            r_t : location of the contact points
            cnt : contact array (1 if in contact, 0 otherwise)
        '''
        self.block_b_x[3:] = np.reshape(X_t[12:] - X_t[3:9], (6,))
        self.block_b_x[5] += self.g*self.dt

        for ee in range(self.n_eff):
            self.block_A_x[3:6, 3*ee:3*ee+3] = cnt[ee]*(self.dt/self.m)*np.identity(3)

            self.block_A_x[6,3*ee+1] = cnt[ee]*(X_t[2] - r_t[ee,2])*self.dt
            self.block_A_x[6,3*ee+2] = -cnt[ee]*(X_t[1] - r_t[ee,1])*self.dt

            self.block_A_x[7,3*ee+0] = -cnt[ee]*(X_t[2] - r_t[ee,2])*self.dt
            self.block_A_x[7,3*ee+2] = cnt[ee]*(X_t[0] - r_t[ee,0])*self.dt
            
            self.block_A_x[8,3*ee+0] = cnt[ee]*(X_t[1] - r_t[ee,1])*self.dt
            self.block_A_x[8,3*ee+1] = -cnt[ee]*(X_t[0] - r_t[ee,0])*self.dt

    def compute_X_mat(self, X, r, cnt_plan):
        '''
        Creates the A(x), b(F) matrix of the entire problem
        Input:
            X : state vector 9*(n_col + 1)
            r : end effector locations (n_col + 1,n_eff, 3)
            cnt_plan : contact plan (n_col + 1, n_eff)
        '''
        # extra 9 for initial condition constraints
        A_x = np.zeros((9*self.n_col + 9, 3*self.n_eff*self.n_col))
        b_x = np.zeros((9*self.n_col + 9, 1))
        
        for t in range(self.n_col):
            self.create_block_x(X[t*9:9*(t+1)+9,0], r[t], cnt_plan[t])
            A_x[t*9:(t+1)*9, 3*self.n_eff*t:3*self.n_eff*(t+1)] = self.block_A_x.copy()
            b_x[t*9:(t+1)*9, 0] = self.block_b_x.copy()

        return np.matrix(A_x), np.matrix(b_x)

    def compute_F_mat(self, F, r, cnt_plan, X_init):
        '''
        Creates the A(F), b(F) matrix of the entire problem
        Input:
            F : state vector n_eff*3*n_col
            r : end effector locations (n_col + 1,n_eff, 3)
            cnt_plan : contact plan (n_col + 1, n_eff)
            X_init : initial conditions of state (9d vector)
        '''
        A_f = np.zeros((9*self.n_col + 9, 9*(self.n_col + 1)))
        b_f = np.zeros((9*self.n_col + 9,1))
        
        for t in range(self.n_col):
            self.create_block_f(F[t*3*self.n_eff: (t+1)*3*self.n_eff], r[t], cnt_plan[t])
            A_f[t*9:(t+1)*9, 9*t:9*(t+1)+9] = self.block_A_f.copy()
            b_f[t*9:(t+1)*9,0] = self.block_b_f.copy()

        A_f[-9:,0:9] = np.identity(9)
        b_f[-9:,0] = X_init

        return np.matrix(A_f), np.matrix(b_f)


        