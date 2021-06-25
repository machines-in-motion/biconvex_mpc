## This is a demo for the A1 trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from matplotlib import pyplot as plt

from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
#from robot_properties_solo.config import Solo12Config
from a1_mpc_gait_gen import A1MpcGaitGen
from py_biconvex_mpc.bullet_utils.a1_mpc_env import A1Env

import subprocess

# subprocess.Popen([r"/home/pshah/Applications/raisim/raisim_ws/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

time.sleep(2)

## robot config and init

urdf_path = "/home/pshah/Applications/raisim_utils/urdf/a1/urdf/a1.urdf"
pinModel = pin.buildModelFromUrdf(urdf_path,
                                  pin.JointModelFreeFlyer())
pinData = pinModel.createData()

st = 0.25
dt = 5e-2
state_wt = np.array([0.] * 3 + [10] * 3 + [5.0] * (pinModel.nv - 6) \
                    + [0.00] * 3 + [0.01] * 3 + [10.0] *(pinModel.nv - 6))

n_eff = 4
m = pin.computeTotalMass(pinModel)
q0 = np.array(
        [0.0, 0.0, 0.31, 0.0, 0.0, 0.0, 1.0]
        + 2 * [0.0, 0.8, -1.6]
        + 2 * [0.0, 0.8, -1.6] )
v0 = pin.utils.zero(pinModel.nv)
x0 = np.concatenate([q0, pin.utils.zero(pinModel.nv)])

v_des = np.array([0.0, 0.0, 0])
sl_arr = v_des*st
t = 0.0
step_height = 0.1

plan_freq = 0.05 # sec
update_time = 0.02 # sec (time of lag)

gg = A1MpcGaitGen(pinModel, pinData, urdf_path, st, dt, state_wt, x0, plan_freq, gait=0)

# while True:
n = 1

sim_t = 0.0
step_t = 0
sim_dt = .001
index = 0
pln_ctr = 0
robot = A1Env(8.0, 0.5, q0, v0, pinModel = pinModel, pinData = pinData, vis_ghost = False)

lag = int(update_time/sim_dt)

for o in range(int(500*(st/sim_dt))):

    # this bit has to be put in shared memory
    if pln_ctr == 0:
        q, v = robot.get_state()
        hip_loc = robot.get_hip_locations()

        next_loc = np.array([[ hip_loc[0][0] + sl_arr[0],  hip_loc[0][1] + sl_arr[1], 0],
                             [ hip_loc[1][0] + sl_arr[0],  hip_loc[1][1] + sl_arr[1], 0],
                             [ hip_loc[2][0] + sl_arr[0],  hip_loc[2][1] + sl_arr[1], 0],
                             [ hip_loc[3][0] + sl_arr[0],  hip_loc[3][1] + sl_arr[1], 0]])

        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), n, next_loc, v_des, step_height, 5e-3, 7e-4, contact_configuration)
        print(f_plan[0])
        time.sleep(1)
        # gg.plot()
        #gg.plot_plan()
        gg.reset()
        pr_et = time.time()
        # print("time", pr_et - pr_st)

    # update of plan
    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt):
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt):
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    # control loop

    contact_configuration = gg.cnt_gait[n%2]
    # contact_configuration = robot.get_current_contacts()

    if np.all((contact_configuration==0)):
        print("flight phase")
    q_des = xs[index][:pinModel.nq].copy()
    dq_des = xs[index][pinModel.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt

    step_t = (step_t + sim_dt)%st
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1
    if np.round(step_t,3) == 0:
        n += 1
    if n > 25 and n < 35:
        rec = np.array([0.0, 0.5, 0])
        sl_arr = rec*st

        robot.robot.rai_robot.setExternalForce(0, [0, 0, 0], [0, 3, 0])
    else:
        rec = v_des
        sl_arr = rec*st


# tmp = np.array(tmp)
# tmp_des = np.array(tmp_des)

# plt.plot(tmp[:,2])
# plt.plot(tmp_des[:,2])

# plt.show()

# gg.plot(tmp)
# gg.plot_joints()
# np.savez("../motion_planner/dat_file/ik", xs = xs)

# gg.plot()



