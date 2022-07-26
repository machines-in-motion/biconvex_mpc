## This is a demo for trot motion in mpc
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_anymal.config import AnymalConfig
from robot_properties_anymal.anymalwrapper import AnymalRobot
from mpc.anymal_cyclic_gen import AnymalMpcGaitGen
from motions.cyclic.anymal_trot import trot
from motions.cyclic.anymal_jump import jump


from envs.pybullet_env import PyBulletEnv
from controllers.robot_id_controller import InverseDynamicsController

## robot config and init
pin_robot = AnymalConfig.buildRobotWrapper()
urdf_path = AnymalConfig.urdf_path

n_eff = 4
q0 = np.array(AnymalConfig.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"]

v_des = np.array([0.2,0.0,0.0])
w_des = 0.0

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

## Motion
gait_params = trot
lag = int(update_time/sim_dt)
gg = AnymalMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)

gg.update_gait_params(gait_params, sim_t)

robot = PyBulletEnv(AnymalRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

plot_time = 0 #Time to start plotting

solve_times = []


# robot.start_recording("anymal_slow_stepping.mp4")

for o in range(int(450*(plan_freq/sim_dt))):
    # this bit has to be put in shared memory
    q, v = robot.get_state()
    contact_configuration = robot.get_current_contacts()

    if o == int(100*(plan_freq/sim_dt)):
        gait_params = trot
        gg.update_gait_params(gait_params, sim_t)
        robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

    if pln_ctr == 0:
        contact_configuration = robot.get_current_contacts()
        
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
        # gg.plot(q,v)
        # Plot if necessary
        # if sim_t >= plot_time:
        # gg.plot_plan(q, v)
            # gg.save_plan("trot")

        pr_et = time.time()
        solve_times.append(pr_et - pr_et)

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
    # tau = robot_id_ctrl.id_joint_torques(q, v, q0, v0, v0, np.zeros(12), contact_configuration)
    robot.send_joint_command(tau)

    time.sleep(0.0005)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1


robot.stop_recording()

