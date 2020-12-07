# This file contains the biconvex motion planner
# Author : Avadesh Meduri  
# Date : 6/12/2020

import os.path
import sys
import numpy as np
from matplotlib import pyplot as plt

curdir = os.path.dirname(__file__)
cdir = os.path.abspath(os.path.join(curdir,'../../python/'))
sys.path.append(cdir)

from dynamics.centroidal import CentroidalDynamics

class BiConvexMP(CentroidalDynamics):

    def __init__(self, m, dt, T, n_eff):
        '''
        This is the Bi Convex motion planner
        Input:
            m : mass of the robot
            dt : discretization of the dynamics
            n_eff : number of end effectors
            T : horizon of plan 
        '''
        
        self.dt = dt
        self.m = m
        self.n_col = int(np.round(T/self.dt))
        self.n_eff = n_eff
        
        # Note : no of collocation points should be computed only once
        # and passed to centroidal dynamics
        super().__init__(m, dt, T, n_eff)

    def create_contact_array(self, cnt_plan):
        '''
        This function creates the contact array [[1/0, 1/0], [..], ...]
        and the contact location array 
        Input:
            cnt_plan : contact plan [1/0, x, y, z, start, end]
        '''
        assert np.shape(cnt_plan)[1] == self.n_eff
        assert np.shape(cnt_plan)[2] == 6

        cnt_arr = np.zeros((self.n_col, self.n_eff))
        r_arr = np.zeros((self.n_col, self.n_eff, 3))

        t_arr = np.zeros(self.n_eff)
        for i in range(len(cnt_plan)):
            for n in range(self.n_eff):
                time_steps = int(np.round((cnt_plan[i][n][5] - cnt_plan[i][n][4])/self.dt, 2))
                cnt_arr[:,n][int(t_arr[n]):int(t_arr[n]+time_steps)] = cnt_plan[i][n][0]
                r_arr[int(t_arr[n]):int(t_arr[n]+time_steps), n] = np.tile(cnt_plan[i][n][1:4], (time_steps,1))
                t_arr[n] += time_steps

        self.cnt_arr = cnt_arr
        self.r_arr = r_arr

    def create_cost_X(self, W_X, W_X_ter, X_ter):
        '''
        Creates the cost matrix Q_X and q_X for the X optimization
        Input:
            W_X : weights on the tracking X
            W_X_ter : terminal weights on the tracking of X
        '''
        assert len(W_X) == len(W_X_ter)
        assert len(W_X) == 9

        self.Q_X = np.zeros((9*(self.n_col + 1), 9*(self.n_col + 1)))    
        np.fill_diagonal(self.Q_X, W_X)
        np.fill_diagonal(self.Q_X[-9:, -9:], W_X_ter)
        # self.q = np.repeat(W_X, self.n_col + 1)
        self.q_X = np.zeros((self.n_col+1)*9)
        self.q_X[-9:] = -2*W_X_ter*X_ter

    def create_cost_F(self, W_F):
        '''
        Creates the cost matrix Q_F and q_F for the F optimization
        Input:
            W_F : weights on the F
        '''
        assert len(W_F) == 3*self.n_eff

        self.Q_F = np.zeros((self.n_col*3*self.n_eff, self.n_col*3*self.n_eff))
        np.fill_diagonal(self.Q_F, W_F)
        self.q_F = np.zeros(self.n_col*3*self.n_eff)
        
    def optimize(self, X_init, X_ter, W_X, W_F, W_X_ter, no_iters, X_wm = None, F_wm = None):
        '''
        This function optimizes the centroidal dynamics trajectory
        Input:
            X_init : initial state vector (com(x,y,z), com_vel(x,y,z), AMOM(x,y,z))
            X_ter : desired terminal state vector at the end of the trajectory
            W_X : running weights on the X optimization
            W_F : running weights on the F optimization
            W_X_ter : terminal weights on the X optimization
            no_iters : number of iterations between x and f
            X_wm : starting X to warm start F optimization
            F_wm : starting F to warm start X optimization
        '''
        # creating cost matrices
        self.create_cost_X(W_X, W_X_ter, X_ter)
        self.create_cost_F(W_F)

        if X_wm:
            X_k = X_wm
        else:
            X_k = np.zeros(9*(self.n_col+1))

        if F_wm:
            F_k = F_wm
        else:
            F_k = np.zeros(3*self.n_col*self.n_eff)
            F_k[2::3] = self.m*9.81
        
        # for k in range(no_iters):
            ## adding initial conditions to dynamic array
            # self.compute_X_mat(X_k, self.r, self.cnt_plan)    