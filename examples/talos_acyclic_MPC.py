## Demo for robot doing a cartwheel
## Author : Avadesh Meduri & Paarth Shah
## Date : 04/08/2021

import time
import numpy as np
import pinocchio as pin
from matplotlib import pyplot as plt

from robot_properties_talos.taloswrapper import TalosRobot
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen2 import TalosAcyclicGen
from mpc.data_plotter import DataRecorder
from robot_properties_talos.config import TalosConfig

from motions.acyclic.talos_jump_MPC import plan

pin_robot = TalosConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data
urdf = TalosConfig.urdf_path

q0 = np.array(TalosConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
         "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]

robot = PyBulletEnv(TalosRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
dr = DataRecorder(pin_robot)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = TalosAcyclicGen(pin_robot, urdf)
q, v = robot.get_state()

if plan.use_offline_nom_traj:
    print("offline trajectory is used.......")
    #perform one full offline planning
    plan.use_offline_nom_traj = False
    mg.update_motion_params(plan, q, 0.)
    mg.optimize(q, v, 0.)
    com_opt = mg.mp.return_opt_com()
    mom_opt = mg.mp.return_opt_mom()
    mg.X_nom_offline = np.zeros((9*plan.n_col), float)
    for i in range(plan.n_col):
        mg.X_nom_offline[9*i:9*i+3] = com_opt[i,:]
        mg.X_nom_offline[9*i+3:9*i+9] = mom_opt[i,:]

    plan.plan_freq = [[.2, 0., plan.T]]
    plan.use_offline_nom_traj = True
    # plan.use_offline_nom_traj = False
    plan.W_X = np.array([1e3, 1e3, 1e3, 1., 1., 1., .1, .1, .1])
    plan.W_X_ter = np.array([0., 0., 0., 1., 1., 1., 0.1, 0.1, 0.1])
    plan.W_F = np.array(24*[0.,])
    plan.n_col = 60


mg.update_motion_params(plan, q, 0.)
# mg.optimize(q, v, 0.)

for o in range(int(2.5*plan.T/sim_dt)):

    contact_configuration = robot.get_current_contacts()
    q, v = robot.get_state()
    kp, kd = mg.get_gains(sim_t)
    robot_id_ctrl.set_gains(kp, kd)

    if pln_ctr == 0 or sim_t == 0:

        xs, us, f = mg.optimize(q, v, sim_t)
        xs = xs[lag:]
        us = us[lag:]
        f = f[lag:]
        index = 0
        dr.record_plan(xs, us, f, sim_t)


        # print(sim_t)
        # mg.plot(q, v, plot_force=True)
        #     mg.save_plan("hifive")
        #     assert False

    # controller
    q_des = xs[index][:pin_robot.model.nq].copy()
    dq_des = xs[index][pin_robot.model.nq:].copy()
    tau = robot_id_ctrl.id_joint_torques(q, v, q_des, dq_des, us[index], f[index], contact_configuration)
    robot.send_joint_command(tau)

    # plotting
    # grf = robot.get_ground_reaction_forces()
    dr.record_data(q, v, tau,  f[index], q_des, dq_des, us[index], f[index])
    time.sleep(0.001)

    sim_t += np.round(sim_dt,3)
    pln_ctr = int((pln_ctr + 1)%(mg.get_plan_freq(sim_t)/sim_dt))
    index += 1

# dr.plot_plans()
# dr.plot(False)
