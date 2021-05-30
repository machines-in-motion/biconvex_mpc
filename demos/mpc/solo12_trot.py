## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config
from mpc_gait_gen import SoloMpcGaitGen
from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import subprocess
subprocess.Popen([r"/home/pshah/Applications/raisim/raisim_ws/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
time.sleep(2)

## robot config and init

pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
st = 0.20
dt = 5e-2
state_wt = np.array([0.] * 3 + [10] * 3 + [5.0] * (pin_robot.model.nv - 6) \
                        + [0.00] * 3 + [0.01] * 3 + [10.0] *(pin_robot.model.nv - 6))

n_eff = 4
m = pin.computeTotalMass(pin_robot.model)
q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.3, 0.0, 0])
sl_arr = v_des*st
t = 0.0
step_height = 0.15


plan_freq = 0.05 # sec

gg = SoloMpcGaitGen(pin_robot, urdf_path, st, dt, state_wt, x0, plan_freq, gait = 1)

# while True:
n = 1

sim_t = 0.0
step_t = 0
sim_dt = .001
index = 0
robot = Solo12Env(2.15, 0.03)

tmp = []
tmp_des = []

for o in range(int(500*(st/sim_dt))):

    next_loc = np.array([[ 0.3946 + n*sl_arr[0],   0.14695 + n*sl_arr[1], 0],
                    [ 0.3946 + n*sl_arr[0],  -0.14695 + n*sl_arr[1], 0],
                    [ 0.0054 + n*sl_arr[0],   0.14695 + n*sl_arr[1], 0],
                    [ 0.0054 + n*sl_arr[0],  -0.14695 + n*sl_arr[1], 0]])

    # this bit has to be put in shared memory
    if index == 0:
        # print(index, step_t)
        q, v = robot.get_state()
        contact_configuration = robot.get_current_contacts()
        # pr_st = time.time()
        xs, us, f = gg.optimize(q, v, np.round(step_t,3), n, next_loc, v_des, step_height, 5e-3, 7e-4, contact_configuration)
        # gg.plot_plan()
        gg.reset()
        # pr_et = time.time()
        # print("time", pr_et - pr_st)

    # control loop
    q, v = robot.get_state()
    contact_configuration = robot.get_current_contacts()

    if np.all((contact_configuration==0)):
        print("flight phase")

    #tmp.append(q)
    q_des = xs[index][:pin_robot.model.nq].copy()
    #tmp_des.append(q_des)
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index])
    sim_t += sim_dt

    print(sim_t)

    #What is this???
    step_t = (step_t + sim_dt)%st
    index = int((index + 1)%(plan_freq/sim_dt))
    #print(index)
    if np.round(step_t,3) == 0:
        n += 1
    # if n < 3:
    #     robot.robot.rai_robot.setExternalForce(0, [0, 0.2, 0], [0, 0, -3]) 

    # if n > 5 and n < 10:
    #     robot.robot.rai_robot.setExternalForce(0, [0, 0, 0], [5, 0, 0]) 

# tmp = np.array(tmp)
# tmp_des = np.array(tmp_des)

# plt.plot(tmp[:,2])
# plt.plot(tmp_des[:,2])

# plt.show()

# gg.plot(tmp)
# gg.plot_joints()
# np.savez("../motion_planner/dat_file/ik", xs = xs)

# gg.plot()



