## Demo for robot doing a cartwheel
## Author : Avadesh Meduri & Paarth Shah
## Date : 04/08/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_atlas.atlaswrapper import AtlasRobot,AtlasConfig
from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv
from mpc.abstract_acyclic_gen_atlas import AtlasAcyclicGen
from mpc.data_plotter import DataRecorder

from motions.acyclic.atlas_jump import plan

pin_robot = AtlasConfig.buildRobotWrapper()
rmodel = pin_robot.model
rdata = pin_robot.data
urdf = AtlasConfig.urdf_path

q0 = np.array(AtlasConfig.initial_configuration)
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
print("\n\n\n com:",pin.centerOfMass(rmodel, rdata, q0, v0))
f_arr = ["leg_right_sole1_fix_joint", "leg_right_sole2_fix_joint", "leg_right_sole3_fix_joint", "leg_right_sole4_fix_joint", \
         "leg_left_sole1_fix_joint", "leg_left_sole2_fix_joint", "leg_left_sole3_fix_joint", "leg_left_sole4_fix_joint"]

robot = PyBulletEnv(AtlasRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
dr = DataRecorder(pin_robot)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
lag = int(update_time/sim_dt)

mg = AtlasAcyclicGen(pin_robot, urdf)
q, v = robot.get_state()
mg.update_motion_params(plan, q, sim_t)

plot_time = np.inf

for o in range(10000):

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

        # if sim_t >= plot_time:
        #     print(mg.cnt_plan[0:3])
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

dr.plot_plans()
# dr.plot(False)
