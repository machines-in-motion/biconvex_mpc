## This file contains functions to add different via points in
## the cost function
## Author : Avadesh Meduri 
## Date : 12/03/2021

import numpy as np
from scipy.sparse import csc_matrix


class BiConvexCosts:

    def __init__(self, n_col, dt, T):
        """
        Input:
            n_col : number of collocation points
            dt : discretization time (sec)
            T : total horizon of plan (sec)
        """
        self.n_col = n_col
        self.dt = dt
        self.T = T

        self.via_points = []
        self.via_point_t = []
        self.via_point_wts = []
        self.ik_mom_opt = None
        self.ik_com_opt = None

    def add_via_point(self, via_point, t, wt):
        """
        adds a desired via point to the cost function
        Input:
            via_point : via point
            t : time in the motion where COM should go through via point
            wt : weight
        """
        self.via_points.append(via_point)
        self.via_point_t.append(int(np.round(t/self.dt, 2)))
        self.via_point_wts.append(wt)

    def add_ik_momentum_cost(self, mom_opt):
        """
        adds momentum task
        Input :
            mom_opt : optimal momentum vector (vel, ang_mom)
        """
        self.ik_mom_opt = mom_opt

    def add_ik_com_cost(self, com_opt):
        """
        adds center of mass tracking task
        Input :     
            com_opt : optimal center of mass 
        """
        self.ik_com_opt = com_opt

    def create_cost_X(self, W_X, W_X_ter, X_ter, X_nom = None):
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

        self.q_X = np.zeros((self.n_col+1)*9)
        self.q_X[:-9] = np.tile(W_X, (self.n_col)) 
        
        if not isinstance(X_nom, np.ndarray):
            X_nom = np.zeros(9*(self.n_col))

        for i in range(len(self.via_points)):
            X_nom[9*self.via_point_t[i]:9*self.via_point_t[i] + 3] = \
                self.via_points[i]
            self.q_X[9*self.via_point_t[i]:9*self.via_point_t[i] + 3] = \
                self.via_point_wts[i]
            for j in range(3):
                self.Q_X[9*self.via_point_t[i] + j, 9*self.via_point_t[i] + j] = \
                    self.via_point_wts[i][j]

        if isinstance(self.ik_mom_opt, np.ndarray):
            for i in range(len(self.ik_mom_opt) - 1):
                X_nom[9*i + 3:9*i + 9] = self.ik_mom_opt[i]
            X_ter[3:] = self.ik_mom_opt[-1]

        if isinstance(self.ik_com_opt, np.ndarray):
            for i in range(len(self.ik_com_opt) - 1):
                X_nom[9*i:9*i + 3] = self.ik_com_opt[i]
            X_ter[0:3] = self.ik_mom_opt[-1]


        self.q_X[:-9] *= -2*X_nom
        self.q_X[-9:] = -2*W_X_ter*X_ter

        self.q_X = np.reshape(self.q_X, (len(self.q_X), 1))
        
        self.Q_X = np.matrix(self.Q_X)
        self.q_X = np.matrix(self.q_X)
        
        # C++
        try:
            print("using C++")
            self.dyn_planer.set_cost_x(csc_matrix(self.Q_X), self.q_X)
        except:
            print("running python version")
    
