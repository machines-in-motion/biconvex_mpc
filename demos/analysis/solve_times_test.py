## This is a script to compute the solve times
## Author : Avadesh Meduri
## Date : 2/11/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_cyclic_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance, bound_turn, trot_turn

from solo_mpc_env import AbstractEnv
from blmc_controllers.robot_id_controller import InverseDynamicsController

from matplotlib import pyplot as plt
plt.rcParams.update({'font.size': 10})

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

v_des = np.array([0.3,0.0,0.0])
w_des = 0.0

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

##
solve_times = []
dyn_viol  = []
col_arr = []
## Motion
gait_params = [trot, jump, bound]

gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)
gg.update_gait_params(gait_params[0], sim_t, ik_hor_ratio=1.0)

robot = AbstractEnv(q0, v0, False, False)

q, v = robot.get_state()
contact_configuration = np.array([1,1,1,1])
for i in range(len(gait_params)):
    col_arr.append([])
    dyn_viol.append([])
    solve_times.append([])
    if gait_params[i].motion_name == "Bound":
        n = 32
    else:
        n = 20
    for j in range(n):
        gait_params[i].gait_horizon = 1.0*(j+1)
        gg.update_gait_params(gait_params[i], sim_t, ik_hor_ratio=1.0)
        gg.kd.compute_solve_times()
        gg.mp.collect_statistics()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
        
        solve_times[i].append(gg.kd.return_solve_times())
        col_arr[i].append(gg.horizon)
        dyn_viol[i].append(gg.mp.return_dyn_viol_hist()[-1])

        # Plot if necessary
        # gg.plot_plan(q, v)



fig, ax = plt.subplots(5,1, sharex = True)


for i in range(len(gait_params)):
    solve_times[i] = np.reshape(solve_times[i], (len(solve_times[i]), 3))

    ax[0].plot(col_arr[i], solve_times[i][:,0], label = str(gait_params[i].motion_name))
    ax[0].scatter(col_arr[i], solve_times[i][:,0])
    ax[0].set_ylabel("Dyn Solve Time [sec]")
    ax[0].grid()
    ax[0].legend(loc="upper right")

    ax[1].plot(col_arr[i], solve_times[i][:,1], label = str(gait_params[i].motion_name))    
    ax[1].scatter(col_arr[i], solve_times[i][:,1])    
    ax[1].set_ylabel("IK Solve Time [sec]")
    ax[1].grid()
    ax[1].legend(loc="upper right")

    ax[2].plot(col_arr[i], solve_times[i][:,2], label = str(gait_params[i].motion_name))
    ax[2].scatter(col_arr[i], solve_times[i][:,2])
    ax[2].set_ylabel("Total Solve Time [sec]")
    ax[2].grid()
    ax[2].legend(loc="upper right")
    
    ax[3].plot(col_arr[i], 100*np.divide(solve_times[i][:,0], solve_times[i][:,2]), label = str(gait_params[i].motion_name))
    ax[3].scatter(col_arr[i], 100*np.divide(solve_times[i][:,0], solve_times[i][:,2]))
    ax[3].set_ylabel("Percentage Dyn vs Kin")
    ax[3].grid()
    ax[3].legend(loc="upper right")

    ax[4].plot(col_arr[i], dyn_viol[i], label = str(gait_params[i].motion_name))
    ax[4].scatter(col_arr[i], dyn_viol[i])
    ax[4].set_ylabel("Dynamics Violation")
    ax[4].grid()
    ax[4].legend(loc="upper right")

ax[4].set_xlabel("number of collocation points")

plt.show()

