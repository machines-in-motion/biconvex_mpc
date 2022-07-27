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


eff_names = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
             "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]


robot = TalosConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

## robot config and init
pin_robot = TalosConfig.buildRobotWrapper()
urdf_path = TalosConfig.urdf_path

q0 = np.array(TalosConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0,0.0])
w_des = 0.0

plan_freq = 0.1 # sec
update_time = 0.0 # sec (time of lag)


kp = np.array(2*[1e2,1e2,1e3,2e3,1e3,1e3] + 
              2*[1e2,] + 
              4*[1e2,] + 3*[1e1,] + 
              4*[1e2,] + 3*[1e1,])
kd = np.array(2*[1e1,1e1,1e1,2e1,2e1,2e1] + 
              2*[1.0e1,] + 
              4*[5e0,] + 3*[1e-1,] + 
              4*[5e0,] + 3*[1e0,])

robot = PyBulletEnv(TalosRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, eff_names)

robot_id_ctrl.set_gains(kp,kd)


q, v = robot.get_state()

# simulation variables
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
lag = 0

# robot.start_recording("talos_walk.mp4")

for o in range(int(150*(plan_freq/sim_dt))):

    # this bit has to be put in shared memory
    q, v = robot.get_state()
    contact_configuration = 8*[1,]

    tau = robot_id_ctrl.id_joint_torques(q, v, q0, v0, v0, np.zeros(3*8), contact_configuration)
    robot.send_joint_command(tau)
    # if pln_ctr == 0:
    #     print("sim_t",sim_t)
    #     gg.plot(q, v, plot_force=True)
    time.sleep(0.0005)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

# robot.stop_recording()
