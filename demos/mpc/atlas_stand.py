## Demo of mpc standing mpc
## Author : Avadesh Meduri
## Date : 6/08/2021

from atlas_mpc_gait_gen import AtlasMpcGaitGen
import time
import numpy as np
import copy
import pinocchio as pin

from robot_properties_atlas.config import AtlasConfig

from py_biconvex_mpc.bullet_utils.atlas_mpc_env import AtlasEnv
import os
import raisimpy as raisim
import subprocess

from atlas_gait_params import still


subprocess.Popen([r"/home/ameduri/devel/raisim/raisimLib/raisimUnity/linux/raisimUnity.x86_64"])

time.sleep(2)

pin_robot = AtlasConfig.buildRobotWrapper()

q0 = np.array([ 0.0, 0.0, 0.95, #base
                0.0, 0.0, 0.0, 1.0, #base quaternion
                0.0, #hip yaw
                0.0487, #hip forward/backward
                0.0, #hip tilt
                0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, #left arm
                0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, #right arm
                0.0, 0.0, 0.0, #left hip abductors
                0.0, 0.0, 0.0, #left knee, ankle tilt fwd, ankle tilt side
                0.0, 0.0, 0.0, #right hip abductors
                0.0, 0.0, 0.0]) #right knee, right ankle tilt fwd, right ankle tilt side;
v0 = np.zeros(pin_robot.model.nv)
x0 = np.concatenate([q0, np.zeros(pin_robot.model.nv)])



dt = 0.05
plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)
sim_t = 0.0
step_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

lag = int(update_time/sim_dt)


gait_params = still
gait_time = gait_params.gait_period


robot = AtlasEnv(gait_params.kp, gait_params.kd, q0, v0, False, False)
gg = AtlasMpcGaitGen(pin_robot, AtlasConfig.urdf_path, dt, gait_params, x0, plan_freq, q0, None)

q_des = q0
dq_des = v0

v_des = np.array([0,0,0])

for o in range(int(500*(plan_freq/sim_dt))):

    q, v = robot.get_state()
    if pln_ctr == 0:
        q, v = robot.get_state()
        # reseting origin (causes scaling issues I think otherwise)
        q[0:2] = 0
        contact_configuration = robot.get_current_contacts()
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(step_t,3), v_des, gait_params.step_ht, contact_configuration)
        # gg.plot_plan()
        gg.reset()
        pr_et = time.time()
        # print("time", pr_et - pr_st)
        # assert False

    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt)-1:
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    # control loop

    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    robot.send_joint_command(q_des, dq_des, us[index], f[index], contact_configuration)
    sim_t += sim_dt
    step_t = (step_t + sim_dt)%gait_time
    
    time.sleep(0.001)
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1
