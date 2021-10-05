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
