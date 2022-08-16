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

if plan.use_offline_traj:
    print("offline trajectory is used.......")
    #perform one full offline planning
    plan.use_offline_traj = False
    mg.update_motion_params(plan, q, 0.)
    mg.optimize(q, v, 0.)
    com_opt = mg.mp.return_opt_com()
    mom_opt = mg.mp.return_opt_mom()
    mg.X_centroidal_offline = np.zeros((9*plan.n_col), float)
    mg.X_kinematics_offline = mg.ik.get_xs()

    for i in range(plan.n_col):
        mg.X_centroidal_offline[9*i:9*i+3] = com_opt[i,:]
        mg.X_centroidal_offline[9*i+3:9*i+9] = mom_opt[i,:]

    plan.use_offline_traj = True
    #new dyn weights for tracking MPC
    plan.plan_freq = [[.1, 0., plan.T]]
    plan.W_X = np.array([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 2e3, 3e+4, 3e+4, 3e+4])
    plan.W_X_ter = 0.*np.array([1e5, 1e5, 1e+5, 1e-1, 1e-1, 2e2, 1e+5, 1e+5, 1e+5])
    plan.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
    plan.n_col = 10
    #new kin weights for tracking MPC
    state_wt_1 = np.array([0., 0., 0. ] + [.0, .0, .0] + [10.] * (rmodel.nv - 6) + \
                          [0.0, 0.0, 0.0] + [.0, 0., .0] + [1.] * (rmodel.nv - 6)
                          )
    plan.state_wt = [np.hstack((state_wt_1, [0, plan.T]))]
    plan.state_scale = [[1e0, 0, plan.T]]
    plan.cent_wt = [[500., 10., 500.], [.1, .1, 5., .1, 100., .1]]
    plan.cnt_wt = 5e4


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
