## motion generator for mpc cartwheel
## Author : Avadesh Meduri
## Date : 5/08/2021

import time
import numpy as np
import pinocchio as pin
import crocoddyl
from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config

import raisimpy as raisim
import subprocess
from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

class CarthwheelGen:

    def __init__(self, q0, v0):

        self.robot = Solo12Config.buildRobotWrapper()
        self.rmodel = self.robot.model
        self.rdata = self.robot.data

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.hip_names = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]

        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))


        self.pin_robot = Solo12Config.buildRobotWrapper()
        self.urdf = Solo12Config.urdf_path

        # move this outside
        self.dt = 5e-2
        self.T = 0.8
        self.rt = 0.3
        self.st = 0.3

        self.x_reg = np.concatenate([q0, pin.utils.zero(self.pin_robot.model.nv)])

        self.swing_wt = [1e4, 1e4]
        self.reg_wt = [1e-2, 7e-5]

        self.state_wt = np.array([0., 0, 100] + [100, 0, 100] + 4*[1e3, 50.0, 20] \
                                + [0.00] * 3 + [10, 10, 10] + [3.5] *(self.pin_robot.model.nv - 6))

        self.x_reg[2] = 0.3

        self.state_wt2 = np.array([0., 0, 1000.0] + [100, 100, 100] + 4*[1e3, 1e2, 50] \
                                + [0.00] * 3 + [10, 10, 10] + [3.5] *(self.pin_robot.model.nv - 6))

        self.x_reg2 = self.x_reg.copy()
        self.x_reg2[3:7] = [0,1,0,0]

        self.x_reg2[7:13] = 2 * [0.0, -np.pi + 0.8, -1.6]
        self.x_reg2[13:19] = 2 * [0.0, -np.pi - 0.8, 1.6]

        self.ctrl_wt = [0, 0, 10] + [1, 1, 1] + [50.0] *(self.pin_robot.model.nv - 6)


        self.cnt_plan_ref = [[[ 1.,      0.3946,   0.14695,  0., 0.,  self.st    ],
                        [ 1.,      0.3946,  -0.14695,  0., 0.,  self.st    ],
                        [ 1.,      0.0054,   0.14695,  0., 0.,  self.st    ],
                        [ 1.,      0.0054,  -0.14695,  0., 0.,  self.st    ]],

                        [[ 0.,      0.3946,   0.14695,  0., self.st, self.st + self.rt   ],
                        [ 0.,      0.3946,  -0.14695,  0., self.st, self.st + self.rt   ],
                        [ 0.,      0.0054,   0.14695,  0., self.st, self.st + self.rt   ],
                        [ 0.,      0.0054,  -0.14695,  0., self.st, self.st + self.rt   ]],

                        [[ 1.,      0.3946,   0.14695,  0., self.st + self.rt, self.T    ],
                        [ 1.,      0.3946,  -0.14695,  0., self.st + self.rt, self.T    ],
                        [ 1.,      0.8054,   0.14695,  0., self.st + self.rt, self.T    ],
                        [ 1.,      0.8054,  -0.14695,  0., self.st + self.rt, self.T    ]]]

        self.cnt_plan_ref = np.array(self.cnt_plan_ref)

        self.prune = False


    def gen_cnt_plan(self, t):

        sl = 2*np.array([0.4, 0])

        if t == 0:
            self.prune = True
            self.cnt_plan = self.cnt_plan_ref.copy()
            for j in range(len(self.ee_frame_id)):
                self.cnt_plan[0][j][1:3] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)[0:2]
                if j >= 2:
                    self.cnt_plan[2][j][1:3] = self.cnt_plan[0][j][1:3] + sl
                else:
                    self.cnt_plan[2][j][1:3] = self.cnt_plan[0][j][1:3]

        if t >= self.st and t < self.st + self.rt and self.prune:
            self.cnt_plan = self.cnt_plan[1:]
            self.prune = False

        elif t >= self.st + self.rt:
            self.cnt_plan = self.cnt_plan_ref.copy()[2:]
            # updating foot location after the flip has happened
            for i in range(len(self.cnt_plan)):
                for j in range(len(self.ee_frame_id)):
                    self.cnt_plan[i][j][1:3] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)[0:2]

        self.cnt_plan[0][:,4] = 0    
        self.cnt_plan[0][:,5] -= t
        
        if len(self.cnt_plan) > 1:
            self.cnt_plan[1][:,4] -= t
            self.cnt_plan[1][:,5] -= t

        if len(self.cnt_plan) > 2:
            self.cnt_plan[2][:,4] -= t
            
        self.cnt_plan[-1][:,5] = self.T

        print(self.cnt_plan)
        return self.cnt_plan

    def generate_plan(self, q, v, t):

        t1 = time.time()

        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        x0 = np.hstack((q, v))

        cnt_plan = self.gen_cnt_plan(t)

        m = pin.computeTotalMass(self.rmodel)

        bx = 0.15
        by = 0.15
        bz = 0.25

        fx_max = 25
        fy_max = 25
        fz_max = 25

        W_X =        np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+3, 1e+4])
        W_X_ter = 10*np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+4, 1e+4])
        W_F = np.array(4*[1e+1, 1e+1, 1e+1])
        
        if t == 0:
            self.nom_ht  = [0.5 + q[0], q[1], 0.5]
        
        rho = 5e+4

        dyn_hor = self.T
        mp = BiConvexMP(m, self.dt, dyn_hor, len(self.eff_names), rho = rho)

        X_init = np.zeros(9)
        pin.computeCentroidalMomentum(self.rmodel, self.rdata)
        X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
        X_init[3:] = np.array(self.rdata.hg)
        X_init[3:6] /= m

        X_nom_tmp = np.zeros((9*int(self.T/self.dt)))
        X_nom_tmp[9*(int(0.5*self.st/self.dt)) + 0::9] = self.nom_ht[0]
        X_nom_tmp[9*(int(0.5*self.st/self.dt)) + 1::9] = self.nom_ht[1]
        X_nom_tmp[9*(int(0.5*self.st/self.dt)) + 2::9] = self.nom_ht[2]

        X_nom_tmp[7::9] = 0.2
        X_nom_tmp[9*(int(self.st/self.dt)) + 7::9] = 0.8
        X_nom_tmp[9*(int((self.st + self.rt)/self.dt)) + 7::9] = 0.1
        X_ter = np.zeros_like(X_init)
        
        X_ter[2] = 0.2
        X_ter[0] = self.nom_ht[0]
        X_ter[1] = self.nom_ht[1]
        
        X_nom = np.zeros_like(X_nom_tmp)
        X_nom[0:len(X_nom[9*int(t/self.dt):])] = X_nom_tmp[9*int(t/self.dt):]
        for b in range(9):
            X_nom[len(X_nom[9*int(t/self.dt):])+b::9] = X_ter[b]
        
        X_nom = X_nom[0:9*int(dyn_hor/self.dt)]
        if int(dyn_hor/self.dt) < self.T:
            X_ter = X_nom[9*int(dyn_hor/self.dt):9*int(dyn_hor/self.dt)+9]

        mp.create_contact_array(np.array(cnt_plan))
        mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
        mp.create_cost_X(W_X, W_X_ter, X_ter, X_nom)
        mp.create_cost_F(W_F)

        com_opt, F_opt, mom_opt = mp.optimize(X_init, 150)
        t2 = time.time()
        
        ik_hor = min(dyn_hor, self.T)

        t3 = time.time()

        ik = InverseKinematics(self.urdf, self.dt, ik_hor)
        
        ik.add_com_position_tracking_task(0, ik_hor, com_opt[0:int(ik_hor/self.dt)], 1, "com", False)
        ik.add_centroidal_momentum_tracking_task(0, ik_hor, mom_opt[0:int(ik_hor/self.dt)], 1e2, "mom", False)

        for i in range(int(ik_hor/self.dt)):
            lt = i*self.dt #local time

            if lt < self.st + self.rt and len(cnt_plan) > 2:
                ik.add_state_regularization_cost_single(i, self.reg_wt[0], "xReg", self.state_wt, self.x_reg)
            else:
                ik.add_state_regularization_cost_single(i, 500*self.reg_wt[0], "xReg2", self.state_wt2, self.x_reg2)
            
            for j in range(len(self.eff_names)):
                for b in range(len(cnt_plan)):
                    if lt < cnt_plan[b][j][5] and lt >= cnt_plan[b][j][4] and cnt_plan[b][j][0] == 1:
                        pos = cnt_plan[b][j][1:4]
                        ik.add_position_tracking_task_single(self.ee_frame_id[j], pos, self.swing_wt[0],
                                                                        "cnt_" + str(0) + self.eff_names[j], i)


        ik.add_ctrl_regularization_cost_2(0, ik_hor, self.reg_wt[1], "uReg", self.ctrl_wt, np.zeros(self.rmodel.nv), True)

        ik.setup_costs()

        ik.optimize(x0)
        t4 = time.time()
        print("dyn :", t2 - t1)
        print("ik :", t4 - t3)
        print("total :", t4 - t1)

        xs = ik.get_xs()
        us = ik.get_us()
            
        n_eff = 3*4

        for i in range(len(xs)-2):
            if i == 0:
                f_int = np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))
                xs_int = np.linspace(xs[i], xs[i+1], int(self.dt/0.001))
                us_int = np.linspace(us[i], us[i+1], int(self.dt/0.001))

                com_int = np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))
                mom_int = np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))
            else:
                f_int =  np.vstack((f_int, np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))))
                xs_int = np.vstack((xs_int, np.linspace(xs[i], xs[i+1], int(self.dt/0.001))))
                us_int = np.vstack((us_int, np.linspace(us[i], us[i+1], int(self.dt/0.001))))

                com_int = np.vstack((com_int, np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))))
                mom_int = np.vstack((mom_int, np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))))


        return xs_int, us_int, f_int