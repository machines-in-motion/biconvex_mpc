## This is a script to compute the Dynamic violation
## Author : Avadesh Meduri
## Date : 2/11/2021import time


import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_cyclic_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance, bound_turn, trot_turn

from solo_mpc_env import AbstractEnv
from robot_id_controller import InverseDynamicsController

from matplotlib import pyplot as plt
plt.rcParams.update({'font.size': 10})

import time
import numpy as np
import pinocchio as pin

from robot_properties_solo.config import Solo12Config
from abstract_cyclic_gen import SoloMpcGaitGen
from solo12_gait_params import trot, walk, air_bound, bound, still, gallop, jump, balance, bound_turn, trot_turn

from demos.mpc.solo_mpc_env import AbstractEnv
from blmc_controllers.robot_id_controller import InverseDynamicsController

from matplotlib import pyplot as plt
plt.rcParams.update({'font.size': 10})

## robot config and init
pin_robot = Solo12Config.buildRobotWrapper()
urdf_path = Solo12Config.urdf_path

n_eff = 4
q0 = np.array(Solo12Config.initial_configuration)
q0[0:2] = 0.0

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

v_des = np.array([0.3,0.0,0.0])
w_des = 0.0

plan_freq = 0.05 # sec
update_time = 0.0 # sec (time of lag)

sim_t = 0.0
sim_dt = .001
index = 0
pln_ctr = 0

##
dyn_viol  = []
## Motion
gait_params = [trot, jump, bound]

gg = SoloMpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)
gg.update_gait_params(gait_params[0], sim_t, ik_hor_ratio=1.0)

robot = AbstractEnv(q0, v0, False, False)

q, v = robot.get_state()
contact_configuration = np.array([1,1,1,1])
for i in range(len(gait_params)):
    dyn_viol.append([])
    n = 10

    gait_params[i].gait_horizon = 1.0*n
    gg.update_gait_params(gait_params[i], sim_t, ik_hor_ratio=1.0)
    gg.kd.compute_solve_times()
    gg.mp.collect_statistics()
    xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t,3), v_des, w_des)

    dyn_viol[i].append(gg.mp.return_dyn_viol_hist())


for i in range(len(dyn_viol)):
    plt.plot(dyn_viol[i][0], label = str(gait_params[i].motion_name))

plt.legend(loc="upper right")
plt.xlabel("Number of Iteration")
plt.ylabel("Dynamic Violation")
plt.grid()
plt.show()