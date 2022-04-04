## This is a demo for analyzing MPC robustness for Solo12
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_cyclic_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance, bound_turn, trot_turn

from solo_mpc_env import AbstractEnv
from robot_id_controller import InverseDynamicsController
import random
import math

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

n_eff = 4

v_des = np.array([0.0,0.0,0.0])
w_des = 0.0

plan_freq = 0.08 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001


pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
gait_params = bound


q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
lag = int(update_time/sim_dt)

robot_env = AbstractEnv(q0, v0, False, False)

robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

num_trials = 5
max_disturbance_vec = np.zeros(num_trials)

for i in range(num_trials):
    time.sleep(1.0)

    pin_robot = Solo12Config.buildRobotWrapper()
    urdf_path = Solo12Config.urdf_path
    gait_params = bound

    gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)
    gg.update_gait_params(gait_params, sim_t)   

    push_index = 0
    push_duration = 500

    unperturbed_time = 0
    time_betw_push = 200

    num_directions = 12
    direction = np.linspace(0,2*math.pi, num_directions)

    magnitude = 1.0
    dir_idx = 0
    index = 0
    pln_ctr = 0
    dir_ctr = 0

    for o in range(int(105000*(plan_freq/sim_dt))):
        q, v = robot_env.get_state()

        if pln_ctr == 0:
            contact_configuration = robot_env.get_current_contacts()
            
            xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
            print(max_disturbance_vec)
        
        # first loop assume that trajectory is planned
        if o < int(plan_freq/sim_dt) - 1:
            xs = xs_plan
            us = us_plan
            f = f_plan

        # second loop onwards lag is taken into account
        elif pln_ctr == lag and o > int(plan_freq/sim_dt)-1:
            # Not the correct logic
            # lag = int((1/sim_dt)*(pr_et - pr_st))
            lag = 0
            xs = xs_plan[lag:]
            us = us_plan[lag:]
            f = f_plan[lag:]
            index = 0

        if q[2] < 0.085 or q[2] > 0.35:
            max_disturbance_vec[i] = magnitude
            robot_env.reset_env()
            time.sleep(1.0)
            break

        if push_index <= push_duration:
            push_index += 1
            robot_env.robot.rai_robot.setExternalForce(0, np.array([0,0,0]), np.array( [magnitude*math.cos(direction[dir_idx]), \
                                                        magnitude*math.sin(direction[dir_idx]), 0]) )
        else:
            unperturbed_time += 1
            if (unperturbed_time == time_betw_push):
                unperturbed_time = 0
                push_index = 0
                dir_idx = random.randint(0,num_directions-1)
                # if dir_idx == num_directions-1:
                #     #dir_idx = 0
                #     magnitude *= 1.1

                dir_ctr += 1
                if dir_ctr == num_directions -1:
                    dir_ctr = 0
                    magnitude *= 1.1
                    max_disturbance_vec[i] = magnitude

        tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy()\
                                    , us[index], f[index], contact_configuration)
        robot_env.send_joint_command(tau)

        sim_t += sim_dt
        pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
        index += 1
    
    #plan_freq = plan_freq / 2



print("done")


