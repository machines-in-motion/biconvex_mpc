## Demo for bolt humanoid doing a jumping motion
## Author : Majid Khadiv
## Date : 05/04/2022

import time
import numpy as np
import pinocchio as pin

from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen import BoltHumanoidAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.bolt_humanoid_stand import plan

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = pin_robot.model
urdf = BoltHumanoidConfig.urdf_path

q0 = np.array(BoltHumanoidConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_ANKLE", "FR_ANKLE", "L_WRIST", "R_WRIST"]

robot = PyBulletEnv(BoltHumanoidRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
dr = DataRecorder(pin_robot)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = BoltHumanoidAcyclicGen(pin_robot, urdf)
q, v = robot.get_state()
mg.update_motion_params(plan, q, sim_t)

plot_time = np.inf

for o in range(int(5/sim_dt)):
    q, v = robot.get_state()
    kp, kd = mg.get_gains(sim_t)
    robot_id_ctrl.set_gains(kp, kd)

    # controller
    q_des = np.array([ 0., 0., 0.35, 0., 0., 0., 1., 0., 0.78539816, -1.57079633, 0., 0.78539816, -1.57079633, 0., 0., 0.])
    dq_des = np.array([ 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
    us = np.array([ 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
    # f = np.zeros(12)
    robot_mass  = pin.computeTotalMass(pin_robot.model, pin_robot.data)
    f = np.array([0., 0., 0.5 * robot_mass * 9.81, 0., 0., 0.5 * robot_mass * 9.81, 0., 0., 0., 0., 0., 0.])
    tau = robot_id_ctrl.id_joint_torques(q, v, q_des, dq_des, us, f, [])
    robot.send_joint_command(tau)

    # plotting
    # grf = robot.get_ground_reaction_forces()
    # dr.record_data(q, v, tau, grf, q_des, dq_des, us[index], f[index])
    time.sleep(0.001)

    sim_t += np.round(sim_dt,3)
    pln_ctr = int((pln_ctr + 1)%(mg.get_plan_freq(sim_t)/sim_dt))
    index += 1

# dr.plot_plans()
# dr.plot(False)
