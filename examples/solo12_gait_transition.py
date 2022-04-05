## This is a demo for trot motion in mpc
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from mpc.abstract_cyclic_gen import SoloMpcGaitGen
from motions.cyclic.solo12_trot import trot
from motions.cyclic.solo12_bound import bound
from motions.cyclic.solo12_jump import jump


from envs.pybullet_env import PyBulletEnv
from controllers.robot_id_controller import InverseDynamicsController

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path
q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

robot = PyBulletEnv(Solo12Robot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)


v_des = np.array([0.3,0.0,0.0])
w_des = 0.0

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
update_time = 0.0 # sec (time of lag)
plan_freq = 0.05 # sec

## Motion
gait_params = trot
lag = int(update_time/sim_dt)
gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)

gg.update_gait_params(gait_params, sim_t)

plot_time = np.inf #Time to start plotting

for o in range(int(500*(plan_freq/sim_dt))):
    q, v = robot.get_state()

    if o == int(50*(plan_freq/sim_dt)):
        gg.update_gait_params(jump, sim_t)

    if o == int(70*(plan_freq/sim_dt)):
        gg.update_gait_params(bound, sim_t)
        v_des = np.array([0.5,0.0,0.0])

    if o == int(100*(plan_freq/sim_dt)):
        gg.update_gait_params(trot, sim_t)
        v_des = np.array([0.5,0.0,0.0])

    # this bit has to be put in shared memory
    if pln_ctr == 0:
        contact_configuration = robot.get_current_contacts()

        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)

        #Plot if necessary
        if sim_t > plot_time:
            gg.plot_plan()

        pr_et = time.time()

    # first loop assume that trajectory is planned
    if o < int(plan_freq/sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq/sim_dt)-1:
        # Not the correct logic
        lag = int((1/sim_dt)*(pr_et - pr_st))
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    # control loop
    ### TODO: Update the gains also during gait transition and add ID controller outside
    robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)
    tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy()\
                                , us[index], f[index], contact_configuration)
    robot.send_joint_command(tau)

    # time.sleep(0.001)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")


