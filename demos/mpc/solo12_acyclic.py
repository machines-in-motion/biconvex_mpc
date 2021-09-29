## This is a demo for trot motion in mpc
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/04/2021

import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from plan_hifive import plan
from abstract_acyclic_gen import SoloAcyclicGen

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])


plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

## Motion
lag = int(update_time/sim_dt)
gg = SoloAcyclicGen(pin_robot, urdf_path, plan_freq)

gg.update_gait_params(plan, sim_t)

robot = Solo12Env(plan.kp, plan.kd, q0, v0, False, False)

plot_time = np.inf #Time to start plotting

for o in range(int(500*(plan_freq/sim_dt))):
    # this bit has to be put in shared memory
    if pln_ctr == 0:
        q, v = robot.get_state()
        contact_configuration = robot.get_current_contacts()

        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3))

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
    robot.send_joint_command(xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy() \
                             , us[index], f[index], contact_configuration)

    time.sleep(0.001)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1

print("done")


