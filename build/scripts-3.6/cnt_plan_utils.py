# This file contains util functions to create contact plans quickly
# Author : Avadesh Meduri
# Date : 11/12/2020

import numpy as np

class SoloCntGen:

    def __init__(self, l, b):
        '''
        Input:
            l : length of base from center
            b : breadth of base from center
        '''
        
        self.init_arr =np.array([[[1,l,b, 0.0,0, 0], [1,l,-b,0.0,0, 0]\
                                ,[1,-l,b,0.0,0,0], [1,-l,-b,0.0,0,0]]])
        
        
    def return_trot_cnt_plan(self, st, sl, i, c_r):
        '''
        Computes contact plan for trot motion for one step
        Input:
            st : step time
            sl : step length (3d)
            i : step number
            c_r : which legs are in contact
        '''
        cnt_plan_n = self.init_arr.copy()
        cnt_plan_n[0,:,4] = i*st
        cnt_plan_n[0,:,5] = (i+1)*st
        cnt_plan_n[0,:,0] = c_r
        cnt_plan_n[0,:,1:4] += sl
        
        return cnt_plan_n 

    def create_trot_plan(self, st, sl, n_steps):
        '''
        create an entire contact plan for a trot motion for n steps
        Input:
            st : step time
            sl : step length (3d)
            n : number of step
        '''
        cnt_plan = self.return_trot_cnt_plan(st, [0,0,0], 0, [1,1,1,1])
        for n in range(1,n_steps+1):
            if  n%2 == 0:
                cnt_plan_n = self.return_trot_cnt_plan(st, n*sl, n, [1,0,1,0])
            else:
                cnt_plan_n = self.return_trot_cnt_plan(st, n*sl, n, [0,1,0,1])
            cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        cnt_plan_n = self.return_trot_cnt_plan(st, (n_steps)*sl, n_steps+1, [1,1,1,1])
        cnt_plan = np.concatenate((cnt_plan, cnt_plan_n), axis = 0)

        return cnt_plan