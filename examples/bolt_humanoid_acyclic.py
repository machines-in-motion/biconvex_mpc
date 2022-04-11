## Demo for bolt humanoid doing a jumping motion
## Author : Majid Khadiv
## Date : 05/04/2022

import time
import numpy as np
import pinocchio as pin

from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen import SoloAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.bolt_humanoid_stand import plan

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = pin_robot.model
urdf = BoltHumanoidConfig.urdf_path

q0 = np.array(BoltHumanoidConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["L_FOOT", "R_FOOT", "L_HAND", "R_HAND"]

robot = PyBulletEnv(BoltHumanoidRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
dr = DataRecorder(pin_robot)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = SoloAcyclicGen(pin_robot, urdf)
q, v = robot.get_state()
mg.update_motion_params(plan, q, sim_t)

plot_time = np.inf

for o in range(int(14/sim_dt)):

    contact_configuration = robot.get_current_contacts()
    q, v = robot.get_state()
    print(q)
    kp, kd = mg.get_gains(sim_t)
    robot_id_ctrl.set_gains(kp, kd)

    if pln_ctr == 0 or sim_t == 0:

        xs, us, f = mg.optimize(q, v, sim_t)
        xs = xs[lag:]
        us = us[lag:]
        f = f[lag:]
        index = 0
        dr.record_plan(xs, us, f, sim_t)

        if sim_t >= plot_time:
            print(mg.cnt_plan[0:3])
            # mg.plot(q, v, plot_force=True)
            mg.save_plan("hifive")
            assert False

    # controller
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    tau = robot_id_ctrl.id_joint_torques(q, v, q_des, dq_des, us[index], f[index], contact_configuration)
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
