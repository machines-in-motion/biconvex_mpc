# This file contains the biconvex motion planner
# Author : Avadesh Meduri  
# Date : 6/12/2020

import numpy as np
from matplotlib import pyplot as plt

from .. dynamics.centroidal import CentroidalDynamics
from ..solvers.fista import FISTA

class BiConvexMP(CentroidalDynamics):

    def __init__(self, m, dt, T, n_eff, rho = 2e+3, L0 = 0.001, beta = 1.1, maxit = 500, tol = 1e-3):
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
        self.tol = tol

        self.cnt_arr = []
        self.r_arr = []
        # Note : no of collocation points should be computed only once
        # and passed to centroidal dynamics
        super().__init__(m, dt, T, n_eff)
        # make the inputs to these parameters that come from outside
        self.fista = FISTA(L0, beta)

        # arrays to store statistics
        self.f_all = [] # history of cost of force optimization
        self.x_all = [] # history of cost of state optimization
        self.dyn_all = [] # history of dynamics constraint violation
        self.total_all = [] # history of total cost

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
        self.q_X = np.zeros(((self.n_col+1)*9,1))
        self.q_X[-9:,0] = -2*W_X_ter*X_ter

        self.Q_X = np.matrix(self.Q_X)
        self.q_X = np.matrix(self.q_X)

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
            X_k = np.ones((9*(self.n_col+1),1))

        if F_wm:
            F_k = F_wm
        else:
            F_k = np.zeros((3*self.n_col*self.n_eff,1))
            F_k[2::3,0] = self.m*9.81

        # penalty of dynamic constraint violation from ADMM
        P_k = np.zeros((9*self.n_col + 9, 1))

        for k in range(no_iters):
            print("iter number {}".format(k), end='\n')
            self.fista.reset()
            # optimizing for f
            A_x, b_x = self.compute_X_mat(X_k, self.r_arr, self.cnt_arr)
            obj_f = lambda f :f.T *self.Q_F*f + self.q_F.T*f + self.rho*np.linalg.norm(A_x*f - b_x + P_k)**2    
            # gradient of the objective function that optimizes for f 
            grad_obj_f = lambda f: 2*self.Q_F*f + self.q_F + 2*self.rho*A_x.T*(A_x*f - b_x + P_k)
            # projection of f into constraint space (friction cone and max f)
            proj_f = lambda f, L : np.clip(f, self.F_low, self.F_high)
            F_k_1 = self.fista.optimize(obj_f, grad_obj_f, proj_f, F_k, self.maxit, self.tol)

            # assert False
            self.fista.reset()
            # optimizing for x
            A_f, b_f = self.compute_F_mat(F_k_1, self.r_arr, self.cnt_arr, X_init)
            obj_x = lambda x :x.T *self.Q_X*x + self.q_X.T*x + self.rho*np.linalg.norm(A_f*x - b_f + P_k)**2    
            # gradient of the objective function that optimizes for f 
            grad_obj_x = lambda x: 2*self.Q_X*x + self.q_X + 2*self.rho*A_f.T*(A_f*x - b_f + P_k)
            # projection of f into constraint space (friction cone and max f)
            proj_x = lambda x, L : np.clip(x, self.X_low, self.X_high)
            X_k_1 = self.fista.optimize(obj_x, grad_obj_x, proj_x, X_k, self.maxit, self.tol)

            # update of P_k
            P_k_1 = P_k + A_f*X_k_1 - b_f

            # preparing for next iteration
            X_k = X_k_1
            F_k = F_k_1
            P_k = P_k_1
            
            # computing cost of total optimization problem
            dyn_violation = np.linalg.norm(A_f*X_k - b_f)**2
            cost_x = X_k.T *self.Q_X*X_k + self.q_X.T*X_k + dyn_violation
            cost_f = F_k.T *self.Q_F*F_k + self.q_F.T*F_k + dyn_violation

            total_cost = cost_x + cost_f - dyn_violation
            self.x_all.append(float(cost_x))            
            self.f_all.append(float(cost_f)) 
            self.dyn_all.append(float(dyn_violation))           
            self.total_all.append(float(total_cost))            

            # if k > 2 and np.abs(self.f_all[-2] - self.f_all[-1]) < self.tol:
            #     break
        self.X_opt = X_k
        self.F_opt = F_k
        
        self.mom_opt = np.zeros((self.n_col + 1, 6))
        for i in range(6):
            self.mom_opt[:,i] = self.X_opt[i+3::9].T

        self.mom_opt[:,0:3] = self.m*self.mom_opt[:,0:3]

        return X_k, F_k, self.mom_opt

    def stats(self):
        print("solver terminated in {} iterations".format(len(self.f_all)))

        fig, ax = plt.subplots(3,1)
        ax[0].plot(self.X_opt[0::9], label = "Cx")
        ax[0].plot(self.X_opt[1::9], label = "Cy")
        ax[0].plot(self.X_opt[2::9], label = "Cz")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(self.X_opt[3::9], label = "Vx")
        ax[1].plot(self.X_opt[4::9], label = "Vy")
        ax[1].plot(self.X_opt[5::9], label = "Vz")
        ax[1].grid()
        ax[1].legend()

        ax[2].plot(self.X_opt[6::9], label = "ang_x")
        ax[2].plot(self.X_opt[7::9], label = "ang_y")
        ax[2].plot(self.X_opt[8::9], label = "ang_z")
        ax[2].grid()
        ax[2].legend()

        fig, ax_mom = plt.subplots(3,1)
        ax_mom[0].plot(self.mom_opt[:,0], label = "lmom_x")
        ax_mom[0].grid()
        ax_mom[0].legend()

        ax_mom[1].plot(self.mom_opt[:,1], label = "lmom_y")
        ax_mom[1].grid()
        ax_mom[1].legend()
        
        ax_mom[2].plot(self.mom_opt[:,2], label = "lmom_z")
        ax_mom[2].grid()
        ax_mom[2].legend()


        fig, ax_f = plt.subplots(self.n_eff,1)
        for n in range(self.n_eff):
            ax_f[n].plot(self.F_opt[3*n::3*self.n_eff], label = "ee: " + str(n) + "Fx")
            ax_f[n].plot(self.F_opt[3*n+1::3*self.n_eff], label = "ee: " + str(n) + "Fy")
            ax_f[n].plot(self.F_opt[3*n+2::3*self.n_eff], label = "ee: " + str(n) + "Fz")
            ax_f[n].grid()
            ax_f[n].legend()

        fig, ax_cost = plt.subplots(4,1)
        ax_cost[0].plot(self.x_all, label = "cost_x")
        ax_cost[0].grid()
        ax_cost[0].legend()

        ax_cost[1].plot(self.f_all, label = "cost_f")
        ax_cost[1].grid()
        ax_cost[1].legend()

        ax_cost[2].plot(self.dyn_all, label = "dynamics_violation")
        ax_cost[2].grid()
        ax_cost[2].legend()

        ax_cost[3].plot(self.total_all, label = "cost_total")
        ax_cost[3].grid()
        ax_cost[3].legend()

        plt.show()