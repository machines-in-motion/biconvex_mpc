## This is a demo for atlas
## Author : Avadesh Meduri
## Date : 06/04/2022

import time
import numpy as np
from mpc.abstract_cyclic_gen1 import AbstractGaitGen
from robot_properties_atlas.config import AtlasConfig
from robot_properties_atlas.atlaswrapper import AtlasRobot

from controllers.robot_id_controller import InverseDynamicsController
from envs.pybullet_env import PyBulletEnv

import pinocchio as pin

import numpy as np
from motions.cyclic.atlas_stand import still

robot = AtlasConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

## robot config and init
pin_robot = AtlasConfig.buildRobotWrapper()
urdf_path = AtlasConfig.urdf_path


eff_names = ["l_foot_lt", "l_foot_rt", "l_foot_lb", "l_foot_rb", "r_foot_lt", "r_foot_rt", "r_foot_lb", "r_foot_rb"]
hip_names = ["l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "l_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz", "r_leg_hpz"]
n_eff = len(eff_names)

q0 = np.array(AtlasConfig.initial_configuration)
q0[0:2] = 0.0
# q0[2] = 1.0
v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])

v_des = np.array([0.0,0.0,0.0])
w_des = 0.0

plan_freq = 0.1 # sec
update_time = 0.0 # sec (time of lag)

gait_params = still

robot = PyBulletEnv(AtlasRobot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, eff_names)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)
gg.update_gait_params(gait_params, 0)

q, v = robot.get_state()

# simulation variables
sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0
lag = 0

for o in range(int(150*(plan_freq/sim_dt))):

    # this bit has to be put in shared memory
    q, v = robot.get_state()
    
    if pln_ctr == 0:
        contact_configuration = len(eff_names)*[1,]
        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)
        # Plot if necessary
        # if sim_t >= plot_time:
        # gg.plot(q, v, plot_force=True)
            # gg.save_plan("trot")
        print(q, v)
        pr_et = time.time()
        # solve_times.append(pr_et - pr_et)

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

    # f[index][2] = 1e3
    tau = robot_id_ctrl.id_joint_torques(q, v, xs[index][:pin_robot.model.nq].copy(), xs[index][pin_robot.model.nq:].copy()\
                                , us[index], f[index], contact_configuration)
    
    robot.send_joint_command(tau)

    time.sleep(0.005)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1)%(plan_freq/sim_dt))
    index += 1



# gg = AbstractGaitGen(urdf_path, eff_names, hip_names, x0, plan_freq, q0)

