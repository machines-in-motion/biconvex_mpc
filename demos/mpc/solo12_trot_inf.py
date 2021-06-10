## This is a demo for creating motions with an infinite horizon

import time
import numpy as np
import pinocchio as pin

from matplotlib import pyplot as plt

from robot_properties_solo.config import Solo12Config
from abstract_mpc_gait_gen import SoloMpcGaitGen
from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import subprocess
from decimal import Decimal

# subprocess.Popen([r"/home/pshah/Applications/raisim/raisim_ws/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64"])
# subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

time.sleep(2)

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
dt = 0.05
state_wt = np.array([0.] * 3 + [10] * 3 + [5.0] * (pin_robot.model.nv - 6) \
                    + [0.00] * 3 + [0.01] * 3 + [10.0] *(pin_robot.model.nv - 6))

q0 = np.array(Solo12Config.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

plan_freq = 0.05 # sec
gg = SoloMpcGaitGen(pin_robot, urdf_path, dt, state_wt, x0, plan_freq)

v_des = np.array([0.0, 0.0, 0])
sl_arr = v_des*gg.stance_percent[0]*gg.gait_period
step_height = 0.1

# while True:
sim_t = 0.0
sim_dt = .001
pln_ctr = 0
robot = Solo12Env(2.5, 0.1, q0, v0, False)

plan_ctr_2 = 0

#lag = int(update_time/sim_dt)

while True:
    # this bit has to be put in shared memory
    #if plan_ctr_2 == 50 or sim_t == 0.0:
    if round((sim_t*1000)) % round((plan_freq*1000)) == 0.0:
        plan_ctr_2 = 0
        q, v = robot.get_state()
        hip_loc = robot.get_hip_locations()
        next_loc = np.array([[ hip_loc[0][0],  hip_loc[0][1], 0],
                             [ hip_loc[1][0],  hip_loc[1][1], 0],
                             [ hip_loc[2][0],  hip_loc[2][1], 0],
                             [ hip_loc[3][0],  hip_loc[3][1], 0]])

        contact_configuration = robot.get_current_contacts()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, sim_t, next_loc, v_des, step_height, 5e-3, 7e-4, contact_configuration)
        contact_configuration = gg.gait_planner.get_phase(sim_t)
        print(contact_configuration)
        #gg.plot_plan()
        gg.reset()

    # control loop
    contact_configuration = gg.gait_planner.get_phase(sim_t)
    #print(contact_configuration)
    q_des = xs_plan[plan_ctr_2][:pin_robot.model.nq].copy()
    dq_des = xs_plan[plan_ctr_2][pin_robot.model.nq:].copy()

    robot.send_joint_command(q_des, dq_des, us_plan[plan_ctr_2], f_plan[plan_ctr_2], contact_configuration)
    sim_t += sim_dt
    plan_ctr_2 += 1


