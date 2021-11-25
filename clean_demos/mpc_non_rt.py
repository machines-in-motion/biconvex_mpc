## Demo file for running MPC
import os

from robot_model.robot_model import RobotModel
from robot_interface.robot_interface import AbstractInterface
from contact_planner.contact_planner import ContactPlanner
from paths.paths import Paths
from controllers.robot_id_controller import InverseDynamicsController

#Load robot
paths = Paths("solo12")

#Initialize Classes
robot_interface = AbstractInterface(paths.URDF_PATH, 0.001, )
contact_planner.initialize(gait_params, height_map)
#kino_dyn.initialize(gait_params)
controller = InverseDynamicsController()

sim_dt = 0.0
dt = paths.YAML["dt"]
steps = paths.YAML["steps"]
for i in range(steps):
	sim_dt += dt

	#Get State
	q, v = robot_interface.get_state()

	#Get Contact Plan
	contact_plan = contact_planner.create_contacts(q, v)

	#Create costs
	costs = kino_dyn.create_costs(sim_dt, q, v, contact_plan)

	#Compute trajectory from kino-dyn
	#Question: Should Lag be incorporated in the kino_dyn.compute_trajectory() function or outside?
	q_des, qd_des, qdd_des = kino_dyn.compute_trajectory(sim_dt, q, v, contact_plan)

	#Visualize Anything we might want
	robot_interface.visualize(contact_plan)

	#Compute torques from controller
	tau = controller.compute_torque(q, v. q_des, qd_des, qdd_des)

	#Send Torque to robot/simulation
	robot_interface.send_torque(tau)