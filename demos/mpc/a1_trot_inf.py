## This is a demo for creating motions with an infinite horizon

import time
import numpy as np
import pinocchio as pin

from matplotlib import pyplot as plt

from abstract_gait_gen import MpcGaitGen
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
dt = 0.05
state_wt = np.array([0.] * 3 + [10] * 3 + [10.0] * (pinModel.nv - 6) \
                    + [0.00] * 3 + [0.01] * 3 + [10.0] *(pinModel.nv - 6))

q0 = np.array(
    [0.0, 0.0, 0.305, 0.0, 0.0, 0.0, 1.0]
    + 2 * [0.0, 0.8, -1.6]
    + 2 * [0.0, 0.8, -1.6])
v0 = pin.utils.zero(pinModel.nv)
x0 = np.concatenate([q0, pin.utils.zero(pinModel.nv)])

plan_freq = .04
gg = MpcGaitGen(pinModel, pinData, urdf_path, dt, state_wt, x0, plan_freq)

v_des = np.array([0.0, 0.0, 0.0])
step_height = 0.125

# while True:
sim_t = 0.0
sim_dt = .001
pln_ctr = 0
robot = A1Env(2.5, 0.3, q0, v0, pinModel = pinModel, pinData = pinData, vis_ghost = False)

#lag = int(update_time/sim_dt)

while True:
    # this bit has to be put in shared memory
    #if plan_ctr_2 == 50 or sim_t == 0.0:
    if round((sim_t*1000)) % round((plan_freq*1000)) == 0.0:
        plan_ctr = 0
        q, v = robot.get_state()
        hip_loc = robot.get_hip_locations()
        next_loc = np.array([[ hip_loc[0][0],  hip_loc[0][1], 0],
                             [ hip_loc[1][0],  hip_loc[1][1], 0],
                             [ hip_loc[2][0],  hip_loc[2][1], 0],
                             [ hip_loc[3][0],  hip_loc[3][1], 0]])
        contact_configuration = gg.gait_planner.get_phase(sim_t)
        xs_plan, us_plan, f_plan = gg.optimize(q, v, round(sim_t*1000)/1000, next_loc, v_des, step_height, 5e-4, 9e-6, contact_configuration)
        contact_configuration = gg.gait_planner.get_phase(sim_t)
        gg.plot_plan()
        if (sim_t > 35.0):
            gg.plot_plan()
        gg.reset()

    # control loop
    #contact_configuration = gg.gait_planner.get_phase(sim_t)
    #print(contact_configuration)
    q_des = xs_plan[plan_ctr][:pinModel.nq].copy()
    dq_des = xs_plan[plan_ctr][pinModel.nq:].copy()

    robot.send_joint_command(q_des, dq_des, us_plan[plan_ctr], f_plan[plan_ctr], contact_configuration)
    sim_t += sim_dt
    plan_ctr += 1


