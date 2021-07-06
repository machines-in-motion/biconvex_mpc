## This is a demo for creating motions with an infinite horizon

import time
import random
import numpy as np
import pinocchio as pin

from abstract_gait_gen import MpcGaitGen
from py_biconvex_mpc.bullet_utils.a1_mpc_env import A1Env

# robot config and init
urdf_path = "/home/pshah/Applications/raisim_utils/urdf/a1/urdf/a1.urdf"
pinModel = pin.buildModelFromUrdf(urdf_path,
                                  pin.JointModelFreeFlyer())
pinData = pinModel.createData()
dt = 0.05
state_wt = np.array([0.] * 3 + [10.0] * 3 + [5.0] * (pinModel.nv - 6) \
                    + [0.00] * 3 + [0.01] * 3 + [1000.0] *(pinModel.nv - 6))
q0 = np.array(
    [0.0, 0.0, 0.305, 0.0, 0.0, 0.0, 1.0]
    + 2 * [0.0, 0.8, -1.6]
    + 2 * [0.0, 0.8, -1.6])
v0 = pin.utils.zero(pinModel.nv)
x0 = np.concatenate([q0, pin.utils.zero(pinModel.nv)]) #Used for regularizing IK
plan_freq = .04

# Set up A1 environment
sim_t = 0.0
sim_dt = .001
pln_ctr = 0
a1_env = A1Env(7.0, 0.4, q0, v0, pinModel=pinModel, pinData=pinData, vis_ghost=True)

# Adding height map information
size = 20.0
samples = 5
terrain = np.zeros((samples, samples))
terrain[1, 1] = 0.5
#height_map = a1_env.env.create_height_map(size, samples, terrain)

# Create gait generation
gg = MpcGaitGen(pinModel, pinData, urdf_path, dt, state_wt, x0, plan_freq, a1_env)

v_des = np.array([0.0, 0.0, 0.0])
step_height = 0.15

#lag = int(update_time/sim_dt)

ee_vis_track = []
for i in range(len(gg.eff_names)):
    ee_vis_track.append(a1_env.env.server.addVisualSphere("v_sphere"+str(i), 0.125, 1, 0, 1, 1))

while True:
    # this bit has to be put in shared memory
    if round((sim_t*1000)) % round((plan_freq*1000)) == 0.0:
        plan_ctr = 0
        q, v = a1_env.get_state()
        hip_loc = a1_env.get_hip_locations()
        next_loc = np.array([[ hip_loc[0][0],  hip_loc[0][1], 0],
                             [ hip_loc[1][0],  hip_loc[1][1], 0],
                             [ hip_loc[2][0],  hip_loc[2][1], 0],
                             [ hip_loc[3][0],  hip_loc[3][1], 0]])
        contact_configuration = gg.gait_planner.get_phase(sim_t)
        xs_plan, us_plan, f_plan = gg.optimize(q, v, round(sim_t*1000)/1000, next_loc, v_des, step_height, 5e-4, 9e-6, contact_configuration)
        contact_configuration = gg.gait_planner.get_phase(sim_t)
        print("Time: ", round((sim_t*1000))/1000)
        #gg.plot_plan()
        # if (sim_t > 0.5):
        #     gg.plot_plan()
        gg.reset()

    # control loop
    q_des = xs_plan[plan_ctr][:pinModel.nq].copy()
    dq_des = xs_plan[plan_ctr][pinModel.nq:].copy()
    a1_env.send_joint_command(q_des, dq_des, us_plan[plan_ctr], f_plan[plan_ctr], contact_configuration)

    for i in range(len(ee_vis_track)):
        ee_vis_track[i].setPosition(np.array([next_loc[i][0] + gg.k_footstep*(v_des[0] - v[0]), next_loc[i][1], next_loc[i][2] ]))

    # if sim_t > 3.0 and sim_t < 3.75:
    #     a1_env.rai_robot.rai_robot.setExternalForce(0, [0, 0, 0], [7, 7, 0])

    sim_t += sim_dt
    plan_ctr += 1


