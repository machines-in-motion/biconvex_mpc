## This file records and plots data based on plan
## Author : Avadesh Meduri
## Date : 4/10/2021

import pinocchio as pin
import numpy as np
from matplotlib import pyplot as plt

class DataRecorder:

    def __init__(self, pin_robot):

        self.rmodel = pin_robot.model
        self.rdata = pin_robot.data

        # arrays
        self.q = []
        self.v = []
        self.a = []
        self.f = []
        self.com = []

        self.des_q = []
        self.des_v = []
        self.des_a = []
        self.des_f = []
        self.des_com = []

        self.xs_plan = []
        self.us_plan = []
        self.f_plan = []
        self.record_time = []

    def record_data(self, q, v, tau, f, des_q, des_v, des_a, des_f):
        """
        This function records data
        """
        self.q.append(q)
        self.v.append(v)
        self.a.append(tau)
        self.f.append(f)
        self.com.append(pin.centerOfMass(self.rmodel, self.rdata, q.copy(), v.copy()))

        self.des_q.append(des_q)
        self.des_v.append(des_v)
        self.des_a.append(des_a)
        self.des_f.append(des_f)
        self.des_com.append(pin.centerOfMass(self.rmodel, self.rdata, des_q.copy(), des_v.copy()))

    def record_plan(self, xs, us, f, sim_t):
        """
        this function stores the plans
        """
        self.xs_plan.append(xs)
        self.us_plan.append(us)
        self.f_plan.append(f)
        self.record_time.append(sim_t)

    def plot_plans(self):
        
        t_arr = 0.001*np.arange(1, 1000*self.record_time[-1] + len(self.xs_plan[-1][:,0])+1)
        self.q = np.asarray(self.q)
        fig, ax = plt.subplots(3,1)
        for i in range(len(self.xs_plan)):
            st = int(1000*self.record_time[i])
            ax[0].plot(t_arr[st:st + len(self.xs_plan[i][:, 0])], self.xs_plan[i][:, 0], label="plan base x - " + str(0.001*st) + " sec")
            ax[1].plot(t_arr[st:st + len(self.xs_plan[i][:, 1])], self.xs_plan[i][:, 1], label="plan base y - " + str(0.001*st) + " sec")
            ax[2].plot(t_arr[st:st + len(self.xs_plan[i][:, 2])], self.xs_plan[i][:, 2], label="plan base z - " + str(0.001*st) + " sec")

        ax[0].plot(t_arr[0:len(self.q[:,0])], self.q[:, 0], label="actual base x")
        ax[1].plot(t_arr[0:len(self.q[:,0])], self.q[:, 1], label="actual base y")
        ax[2].plot(t_arr[0:len(self.q[:,0])], self.q[:, 2], label="actual base z")
        
        ax[0].grid()
        ax[0].legend()

        ax[1].grid()
        ax[1].legend()

        ax[2].grid()
        ax[2].legend()

        eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        
        fig, ax_f = plt.subplots(4, 1, sharex=True, sharey=True)
        t_arr = 0.001*np.arange(1, 1000*self.record_time[-1] + len(self.f_plan[-1][:,0])+1)
        for i in range(len(self.f_plan)):
            for n in range(4):
                st = int(1000*self.record_time[i])
                # ax_f[n].plot(t_arr[st:st+len(self.f_plan[i][:,3*n])], self.f_plan[i][:,3*n],   label = eff_names[n] + " Fx")
                # ax_f[n].plot(t_arr[st:st+len(self.f_plan[i][:,3*n])], self.f_plan[i][:,3*n+1], label = eff_names[n] + " Fy")
                ax_f[n].plot(t_arr[st:st+len(self.f_plan[i][:,3*n])], self.f_plan[i][:,3*n+2], label = eff_names[n] + " Fz")

                ax_f[n].grid()
                ax_f[n].legend()
        
        F_arr = np.asarray(self.f)
        for n in range(4):     
            # ax_f[n].plot(t_arr[0:len(F_arr[:,3*n])], F_arr[:,3*n], label = "real " + eff_names[n] + " Fx")
            # ax_f[n].plot(t_arr[0:len(F_arr[:,3*n])], F_arr[:,3*n+1], label = "real " + eff_names[n] + " Fy")
            ax_f[n].plot(t_arr[0:len(F_arr[:,3*n])], F_arr[:,3*n+2], label = "real " + eff_names[n] + " Fz")


        plt.show()

    def plot(self, plot_force = True):

        F_arr = np.asarray(self.f)
        f_des = np.asarray(self.des_f)
        com = np.asarray(self.com)
        des_com = np.asarray(self.des_com)
        
        fig, ax = plt.subplots(3,1)
        ax[0].plot(com[:, 0], label="actual com x")
        ax[0].plot(des_com[:, 0], label="des com x")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(com[:, 1], label="actual com y")
        ax[1].plot(des_com[:, 1], label="des com y")
        ax[1].grid()
        ax[1].legend()

        ax[2].plot(com[:, 2], label="actual com z")
        ax[2].plot(des_com[:, 2], label="des com z")
        ax[2].grid()
        ax[2].legend()
        
        eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        
        fig, ax_f = plt.subplots(4, 1, sharex=True, sharey=True)
        for n in range(4):
            ax_f[n].plot(f_des[:,3*n], label = eff_names[n] + " Fx")
            ax_f[n].plot(f_des[:,3*n+1], label = eff_names[n] + " Fy")
            ax_f[n].plot(f_des[:,3*n+2], label = eff_names[n] + " Fz")

            ax_f[n].plot(F_arr[:,3*n], label = "real " + eff_names[n] + " Fx")
            ax_f[n].plot(F_arr[:,3*n+1], label = "real " + eff_names[n] + " Fy")
            ax_f[n].plot(F_arr[:,3*n+2], label = "real " + eff_names[n] + " Fz")

            ax_f[n].grid()
            ax_f[n].legend()
        
        plt.show()
