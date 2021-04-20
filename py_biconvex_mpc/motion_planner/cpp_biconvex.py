# This file contains the biconvex motion planner with C++
# Author : Avadesh Meduri & Paarth Shah
# Date : 9/4/2021

import time
import numpy as np
from matplotlib import pyplot as plt
from scipy.sparse import csc_matrix

from biconvex_mpc_cpp import BiconvexMP
from . cost import BiConvexCosts

class BiConvexMP(BiConvexCosts):

    def __init__(self, m, dt, T, n_eff, rho = 1e+5, L0 = 1e+2, beta = 1.5, maxit = 150, tol = 1e-5):
        '''
        This is the Bi Convex motion planner
        Input:
            m : mass of the robot
            dt : discretization of the dynamics
            n_eff : number of end effectors
            T : horizon of plan 
            rho : penalty on dynamic constraint violation
            L0 : starting step length size during line search
            beta : increas factor of step length
            maxit : maximum number of iterations each component of the biconvex optimization is run
            tol : exit criteria of the optimization
        '''
        
        self.dt = dt
        self.m = m
        self.n_col = int(np.round(T/self.dt, 2))
        self.n_eff = n_eff
        self.rho = rho
        self.maxit = maxit
        self.beta = beta
        self.L0 = L0
        self.tol = tol
        self.cnt_arr = []
        self.r_arr = []
        # Note : no of collocation points should be computed only once
        # and passed to centroidal dynamics
        BiConvexCosts.__init__(self, self.n_col, self.dt, T)
        
        # C++ version Biconvex MP
        self.dyn_planer = BiconvexMP(self.m, self.dt, self.T, self.n_eff)

        
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

        # This should be removed ASAP

        t_arr = np.zeros(self.n_eff)
        for i in range(len(cnt_plan)):
            for n in range(self.n_eff):
                time_steps = int(np.round((cnt_plan[i][n][5] - cnt_plan[i][n][4])/self.dt, 2))
                cnt_arr[:,n][int(t_arr[n]):int(t_arr[n]+time_steps)] = cnt_plan[i][n][0]
                r_arr[int(t_arr[n]):int(t_arr[n]+time_steps), n] = np.tile(cnt_plan[i][n][1:4], (time_steps,1))
                t_arr[n] += time_steps

        self.cnt_arr = cnt_arr
        self.r_arr = r_arr

        # C++ input 
        for i in range(cnt_plan.shape[0]):
            self.dyn_planer.set_contact_plan(cnt_plan[i])
        
        self.dyn_planer.create_contact_array()

    def create_bound_constraints(self, bx, by, bz, fx_max, fy_max, fz_max):
        '''
        Creates arrays that are needed for ensuring kinematic constraints
        Input:
            bx : maximum dispacement of x from the center of the kinematic box
            by : maximum dispacement of y from the center of the kinematic box
            bz : maximum dispacement of z from ground
            fx_max : max force in x
            fy_max : max force in y
            fz_max : max_force in z
        '''
        # shape the arrays properly at once
        self.F_low = np.tile([-fx_max, -fy_max, 0.0], self.n_eff*self.n_col)
        self.F_high = np.tile([fx_max, fy_max, fz_max], self.n_eff*self.n_col)
        
        self.X_low = -999999*np.ones(9*self.n_col + 9)
        self.X_high = 999999*np.ones(9*self.n_col + 9)

        for i in range(len(self.r_arr)):
            if np.sum(self.cnt_arr[i]) > 0:
                self.X_low[9*i:9*i+3] = [-bx, -by, 0]
                self.X_high[9*i:9*i+3] = [bx, by, bz]
            for n in range(self.n_eff):
                if np.sum(self.cnt_arr[i]) > 0:
                    self.X_low[9*i:9*i+3] += self.cnt_arr[i,n]*self.r_arr[i,n]/np.sum(self.cnt_arr[i]) 
                    self.X_high[9*i:9*i+3] += self.cnt_arr[i,n]*self.r_arr[i,n]/np.sum(self.cnt_arr[i]) 
        self.X_low[-9:] = self.X_low[-18:-9]
        self.X_high[-9:] = self.X_high[-18:-9]

        self.F_low = np.reshape(self.F_low, (len(self.F_low), 1))
        self.F_high = np.reshape(self.F_high, (len(self.F_high), 1))
        
        self.X_low = np.reshape(self.X_low, (len(self.X_low), 1))
        self.X_high = np.reshape(self.X_high, (len(self.X_high), 1))

        ## C++ 
        self.dyn_planer.set_bounds_x(self.X_low, self.X_high)
        self.dyn_planer.set_bounds_f(self.F_low, self.F_high)


    def create_cost_F(self, W_F):
        '''
        Creates the cost matrix Q_F and q_F for the F optimization
        Input:
            W_F : weights on the F
        '''
        assert len(W_F) == 3*self.n_eff

        self.Q_F = np.zeros((self.n_col*3*self.n_eff, self.n_col*3*self.n_eff))
        np.fill_diagonal(self.Q_F, W_F)
        self.q_F = np.zeros((self.n_col*3*self.n_eff, 1))

        self.Q_F = np.matrix(self.Q_F)
        self.q_F = np.matrix(self.q_F)

        # C++
        self.dyn_planer.set_cost_f(csc_matrix(self.Q_F), self.q_F)

    def optimize(self, X_init, no_iters, X_wm = None, F_wm = None, P_wm = None):
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

        if isinstance(X_wm, np.ndarray):
            X_k = X_wm
        else:
            X_k = np.tile(X_init, ((self.n_col+1)))[:,None]

        if isinstance(F_wm, np.ndarray):
            F_k = F_wm
        else:
            F_k = np.zeros((3*self.n_col*self.n_eff,1))
        
        # penalty of dynamic constraint violation from ADMM
        if isinstance(P_wm, np.ndarray):
            P_k = P_wm
        else:    
            P_k = 0.0*np.ones((9*self.n_col + 9, 1))

        # C++        
        self.dyn_planer.set_warm_start_vars(X_k, F_k, P_k)
        self.dyn_planer.optimize(X_init, no_iters)

        self.X_opt = self.dyn_planer.return_opt_x()
        self.F_opt = self.dyn_planer.return_opt_f()
        self.P_opt = self.dyn_planer.return_opt_p()

        com_opt = np.zeros((self.n_col + 1, 3))
        self.mom_opt = np.zeros((self.n_col + 1, 6))
        for i in range(6):
            if i < 3:
                com_opt[:,i] = self.X_opt[i::9].T
            self.mom_opt[:,i] = self.X_opt[i+3::9].T

        self.mom_opt[:,0:3] = self.m*self.mom_opt[:,0:3]

        return com_opt, F_k, self.mom_opt

    def get_optimal_x_p(self):

        return self.X_opt, self.P_opt

    def stats(self):

        fig, ax = plt.subplots(3,1)
        ax[0].plot(self.X_opt[0::9], label = "Cx")
        ax[0].plot(self.X_opt[1::9], label = "Cy")
        ax[0].plot(self.X_opt[2::9], label = "Cz")

        # ax[0].plot(self.X_opt_old[0::9], label = "Cx_old")
        # ax[0].plot(self.X_opt_old[1::9], label = "Cy_old")
        # ax[0].plot(self.X_opt_old[2::9], label = "Cz_old")


        if isinstance(self.ik_com_opt, np.ndarray):
            ax[0].plot(self.ik_com_opt[:,0], label = "ik_Cx")
            ax[0].plot(self.ik_com_opt[:,1], label = "ik_Cy")
            ax[0].plot(self.ik_com_opt[:,2], label = "ik_Cz")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(self.X_opt[3::9], label = "Vx")
        ax[1].plot(self.X_opt[4::9], label = "Vy")
        ax[1].plot(self.X_opt[5::9], label = "Vz")

        # ax[1].plot(self.X_opt_old[3::9], label = "Vx_old")
        # ax[1].plot(self.X_opt_old[4::9], label = "Vy_old")
        # ax[1].plot(self.X_opt_old[5::9], label = "Vz_old")

        if isinstance(self.ik_mom_opt, np.ndarray):
            ax[1].plot(self.ik_mom_opt[:,0], label = "ik_Vx")
            ax[1].plot(self.ik_mom_opt[:,1], label = "ik_Vy")
            ax[1].plot(self.ik_mom_opt[:,2], label = "ik_Vz")
            
        ax[1].grid()
        ax[1].legend()

        ax[2].plot(self.X_opt[6::9], label = "ang_x")
        ax[2].plot(self.X_opt[7::9], label = "ang_y")
        ax[2].plot(self.X_opt[8::9], label = "ang_z")
        # ax[2].plot(self.X_opt_old[6::9], label = "ang_x_old")
        # ax[2].plot(self.X_opt_old[7::9], label = "ang_y_old")
        # ax[2].plot(self.X_opt_old[8::9], label = "ang_z_old")

        if isinstance(self.ik_mom_opt, np.ndarray):
            ax[2].plot(self.ik_mom_opt[:,3], label = "ik_ang_x")
            ax[2].plot(self.ik_mom_opt[:,4], label = "ik_ang_y")
            ax[2].plot(self.ik_mom_opt[:,5], label = "ik_ang_z")
            
        ax[2].grid()
        ax[2].legend()

        # fig, ax_mom = plt.subplots(3,1)
        # ax_mom[0].plot(self.mom_opt[:,0], label = "lmom_x")
        # ax_mom[0].grid()
        # ax_mom[0].legend()

        # ax_mom[1].plot(self.mom_opt[:,1], label = "lmom_y")
        # ax_mom[1].grid()
        # ax_mom[1].legend()
        
        # ax_mom[2].plot(self.mom_opt[:,2], label = "lmom_z")
        # ax_mom[2].grid()
        # ax_mom[2].legend()


        fig, ax_f = plt.subplots(self.n_eff,1)
        for n in range(self.n_eff):
            ax_f[n].plot(self.F_opt[3*n::3*self.n_eff], label = "ee: " + str(n) + "Fx")
            ax_f[n].plot(self.F_opt[3*n+1::3*self.n_eff], label = "ee: " + str(n) + "Fy")
            ax_f[n].plot(self.F_opt[3*n+2::3*self.n_eff], label = "ee: " + str(n) + "Fz")

            # ax_f[n].plot(self.F_opt_old[3*n::3*self.n_eff], label = "old_ee: " + str(n) + "Fx")
            # ax_f[n].plot(self.F_opt_old[3*n+1::3*self.n_eff], label = "old_ee: " + str(n) + "Fy")
            # ax_f[n].plot(self.F_opt_old[3*n+2::3*self.n_eff], label = "old_ee: " + str(n) + "Fz")

            ax_f[n].grid()
            ax_f[n].legend()

        # fig, ax_cost = plt.subplots(4,1)
        # ax_cost[0].plot(self.x_all, label = "cost_x")
        # ax_cost[0].grid()
        # ax_cost[0].legend()

        # ax_cost[1].plot(self.f_all, label = "cost_f")
        # ax_cost[1].grid()
        # ax_cost[1].legend()

        # ax_cost[2].plot(self.dyn_all, label = "dynamics_violation")
        # ax_cost[2].grid()
        # ax_cost[2].legend()

        # ax_cost[3].plot(self.total_all, label = "cost_total")
        # ax_cost[3].grid()
        # ax_cost[3].legend()

        plt.show()