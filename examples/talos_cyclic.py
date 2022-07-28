## This is a demo for atlas
## Author : Avadesh Meduri
## Date : 06/04/2022

import time
import numpy as np
from robot_properties_talos.config import TalosConfig
from robot_properties_talos.taloswrapper import TalosRobot
from mpc.abstract_cyclic_gen1 import AbstractGaitGen

from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv

import pinocchio as pin

import numpy as np
from motions.cyclic.talos_stand import still
from motions.cyclic.talos_walk import walk
from motions.cyclic.talos_jump import jump


robot = TalosConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

## robot config and init
pin_robot = TalosConfig.buildRobotWrapper()
urdf_path = TalosConfig.urdf_path


eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
             "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]
hip_names = 4*["leg_right_1_joint",] + 4*["leg_left_1_joint",]
n_eff = len(eff_names)

q0 = np.array(TalosConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0,0.0])
w_des = 0.0

plan_freq = 1.0 # sec
update_time = 0.0 # sec (time of lag)

gait_params = jump


robot = PyBulletEnv(TalosRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, eff_names)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)
gg.update_gait_params(gait_params, 0)

q, v = robot.get_state()

# simulation variables
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
lag = 0

# robot.start_recording("talos_jump.mp4")

for o in range(int(150*(plan_freq/sim_dt))):

    # this bit has to be put in shared memory
    q, v = robot.get_state()
    if pln_ctr == 0:
        # print("time: ", o/1000)
        # print(q, v)
        contact_configuration = len(eff_names)*[1,]
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
        # Plot if necessary
        # if sim_t >= plot_time:
        # if o/1000 > 1:
        # gg.plot_plan(q, v, plot_force=True)
            # gg.save_plan("trot")

        pr_et = time.time()
        # solve_times.append(pr_et - pr_et)

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

    tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy()\
                                , us[index], f[index], contact_configuration)

    # tau = robot_id_ctrl.id_joint_torques(q, v, q0, v0, v0, np.zeros(3*4), contact_configuration)
    # tau[3:] = 0
    robot.send_joint_command(tau)
    # if pln_ctr == 0:
    #     print("sim_t",sim_t)
    #     gg.plot(q, v, plot_force=True)
    time.sleep(0.0005)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

# robot.stop_recording()
