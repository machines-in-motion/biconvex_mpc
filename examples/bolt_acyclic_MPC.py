## Demo for bolt jump
## Author : Majid Khadiv
## Date : 04/08/2022

import time
import numpy as np
import pinocchio as pin
from matplotlib import pyplot as plt

from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen_bolt import BoltHumanoidAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.bolt_jump import plan

pin_robot = BoltHumanoidConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data
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

if plan.use_offline_centroidal_traj:
    print("offline trajectory is used.......")
    #perform one full offline planning
    plan.use_offline_centroidal_traj = False
    mg.update_motion_params(plan, q, 0.)
    mg.optimize(q, v, 0.)
    com_opt = mg.mp.return_opt_com()
    mom_opt = mg.mp.return_opt_mom()
    mg.X_centroidal_offline = np.zeros((9*plan.n_col), float)
    for i in range(plan.n_col):
        mg.X_centroidal_offline[9*i:9*i+3] = com_opt[i,:]
        mg.X_centroidal_offline[9*i+3:9*i+9] = mom_opt[i,:]

    plan.plan_freq = [[.1, 0., plan.T]]
    plan.use_offline_centroidal_traj = True
    # plan.use_offline_centroidal_traj = False
    plan.W_X = np.array([1e3, 1e0, 1e3, .1, .1, 1., .0, .1, .0])
    plan.W_X_ter = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
    plan.W_F = np.array(24*[0.,])
    plan.n_col = 60
    plan.state_scale = [[2e-2, 0, plan.T]]


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
        mg.plot(q, v, plot_force=True)
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
