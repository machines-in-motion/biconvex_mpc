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


robot = Solo12Config.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

pin_robot = Solo12Config.buildRobotWrapper()
urdf = Solo12Config.urdf_path


q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x_reg = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
hip_names = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]
    


def gen_cnt_plan(rdata, fid, t, st, rt, T):

    cnt_plan = [[[ 1.,      0.3946,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.3946,  -0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,   0.14695,  0., 0.,  st    ],
                [ 1.,      0.0054,  -0.14695,  0., 0.,  st    ]],

               [[ 0.,      0.3946,   0.14695,  0., st, st + rt   ],
                [ 0.,      0.3946,  -0.14695,  0., st, st + rt   ],
                [ 0.,      0.0054,   0.14695,  0., st, st + rt   ],
                [ 0.,      0.0054,  -0.14695,  0., st, st + rt   ]],

               [[ 1.,      0.3946,   0.14695,  0., st + rt, T    ],
                [ 1.,      0.3946,  -0.14695,  0., st + rt, T    ],
                [ 1.,      0.8054,   0.14695,  0., st + rt, T    ],
                [ 1.,      0.8054,  -0.14695,  0., st + rt, T    ]]]

    cnt_plan = np.array(cnt_plan)

    cnt_plan[0][:,5] -= t
    cnt_plan[1][:,4] -= t
    cnt_plan[1][:,5] -= t
    cnt_plan[2][:,4] -= t

    if t == 0:
        for j in range(len(fid)):
            cnt_plan[0][j][1:3] = np.round(rdata.oMf[fid[j]].translation, 3)[0:2]

    if t >= st and t < st + rt:
        cnt_plan = cnt_plan[1:]

    elif t >= st + rt:
        cnt_plan = cnt_plan[2:]
        # updating foot location after the flip has happened
        for i in range(len(cnt_plan)):
            for j in range(len(fid)):
                cnt_plan[i][j][1:3] = np.round(rdata.oMf[fid[j]].translation, 3)[0:2]

    cnt_plan[0][:,4] = 0    



    return cnt_plan


## make this a class
def generate_plan(q, v, t):
    t1 = time.time()

    pin.forwardKinematics(rmodel, rdata, q, v)
    pin.updateFramePlacements(rmodel, rdata)

    x0 = np.hstack((q, v))

    ee_frame_id = []
    for i in range(len(eff_names)):
        ee_frame_id.append(rmodel.getFrameId(eff_names[i]))

    dt = 5e-2
    T = 0.8
    rt = 0.3
    st = 0.3

    swing_wt = [1e4, 1e4]
    reg_wt = [1e-2, 7e-5]

    state_wt = np.array([0., 0, 100] + [100, 0, 100] + 4*[1e3, 50.0, 20] \
                            + [0.00] * 3 + [10, 10, 10] + [3.5] *(pin_robot.model.nv - 6))

    x_reg[2] = 0.3

    state_wt2 = np.array([0., 0, 1000.0] + [100, 100, 100] + 4*[1e3, 1e2, 50] \
                            + [0.00] * 3 + [10, 10, 10] + [3.5] *(pin_robot.model.nv - 6))

    x_reg2 = x_reg.copy()
    x_reg2[3:7] = [0,1,0,0]

    x_reg2[7:13] = 2 * [0.0, -np.pi + 0.8, -1.6]
    x_reg2[13:19] = 2 * [0.0, -np.pi - 0.8, 1.6]

    ctrl_wt = [0, 0, 10] + [1, 1, 1] + [50.0] *(pin_robot.model.nv - 6)

    cnt_plan = gen_cnt_plan(rdata, ee_frame_id, t, st, rt, T)

    m = pin.computeTotalMass(rmodel)

    bx = 0.15
    by = 0.15
    bz = 0.25

    fx_max = 25
    fy_max = 25
    fz_max = 25

    W_X =        np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+3, 1e+4])
    W_X_ter = 10*np.array([1e-2, 1e-2, 1e+5, 1e-2, 1e-2, 1e-4, 1e+3, 1e+4, 1e+4])
    W_F = np.array(4*[1e+1, 1e+1, 1e+1])
    nom_ht  = [0.5, 0, 0.5]
    rho = 5e+4

    dyn_hor = T
    mp = BiConvexMP(m, dt, dyn_hor, len(eff_names), rho = rho)

    X_init = np.zeros(9)
    pin.computeCentroidalMomentum(rmodel, rdata)
    X_init[0:3] = pin.centerOfMass(rmodel, rdata, q, v)
    X_init[3:] = np.array(rdata.hg)
    X_init[3:6] /= m

    X_nom_tmp = np.zeros((9*int(T/dt)))
    X_nom_tmp[9*(int(0.5*st/dt)) + 2::9] = nom_ht[2]

    X_nom_tmp[7::9] = 0.2
    X_nom_tmp[9*(int(st/dt)) + 7::9] = 0.8
    X_nom_tmp[9*(int((st + rt)/dt)) + 7::9] = 0.1
    X_ter = np.zeros_like(X_init)
    X_ter[2] = 0.2
    X_ter[0] = 0.5
    
    X_nom = np.zeros_like(X_nom_tmp)
    X_nom[0:len(X_nom[9*int(t/dt):])] = X_nom_tmp[9*int(t/dt):]
    for b in range(9):
        X_nom[len(X_nom[9*int(t/dt):])+b::9] = X_ter[b]
    
    X_nom = X_nom[0:9*int(dyn_hor/dt)]
    if int(dyn_hor/dt) < T:
        X_ter = X_nom[9*int(dyn_hor/dt):9*int(dyn_hor/dt)+9]

    mp.create_contact_array(np.array(cnt_plan))
    mp.create_bound_constraints(bx, by, bz, fx_max, fy_max, fz_max)
    mp.create_cost_X(W_X, W_X_ter, X_ter, X_nom)
    mp.create_cost_F(W_F)

    com_opt, F_opt, mom_opt = mp.optimize(X_init, 150)
    t2 = time.time()
    
    ik_hor = min(dyn_hor, T)

    t3 = time.time()

    ik = InverseKinematics(urdf, dt, ik_hor)
    
    ik.add_com_position_tracking_task(0, ik_hor, com_opt[0:int(ik_hor/dt)], 1, "com", False)
    ik.add_centroidal_momentum_tracking_task(0, ik_hor, mom_opt[0:int(ik_hor/dt)], 1e2, "mom", False)

    for i in range(int(ik_hor/dt)):
        lt = i*dt #local time

        if lt < st + rt and len(cnt_plan) > 2:
            ik.add_state_regularization_cost_single(i, reg_wt[0], "xReg", state_wt, x_reg)
        else:
            ik.add_state_regularization_cost_single(i, 500*reg_wt[0], "xReg2", state_wt2, x_reg2)
        
        for j in range(len(eff_names)):
            for b in range(len(cnt_plan)):
                if lt < cnt_plan[b][j][5] and lt >= cnt_plan[b][j][4] and cnt_plan[b][j][0] == 1:
                    pos = cnt_plan[b][j][1:4]
                    ik.add_position_tracking_task_single(ee_frame_id[j], pos, swing_wt[0],
                                                                    "cnt_" + str(0) + eff_names[j], i)


    ik.add_ctrl_regularization_cost_2(0, ik_hor, reg_wt[1], "uReg", ctrl_wt, np.zeros(rmodel.nv), True)

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
            f_int = np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(dt/0.001))
            xs_int = np.linspace(xs[i], xs[i+1], int(dt/0.001))
            us_int = np.linspace(us[i], us[i+1], int(dt/0.001))

            com_int = np.linspace(com_opt[i], com_opt[i+1], int(dt/0.001))
            mom_int = np.linspace(mom_opt[i], mom_opt[i+1], int(dt/0.001))
        else:
            f_int =  np.vstack((f_int, np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(dt/0.001))))
            xs_int = np.vstack((xs_int, np.linspace(xs[i], xs[i+1], int(dt/0.001))))
            us_int = np.vstack((us_int, np.linspace(us[i], us[i+1], int(dt/0.001))))

            com_int = np.vstack((com_int, np.linspace(com_opt[i], com_opt[i+1], int(dt/0.001))))
            mom_int = np.vstack((mom_int, np.linspace(mom_opt[i], mom_opt[i+1], int(dt/0.001))))


    return xs_int, us_int, f_int