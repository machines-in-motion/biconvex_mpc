## Demo file for running MPC

simulation_steps = 1091234982374982
sim_dt = 0.0
dt = 0.001

#Initialize Classes
robot_interface = RobotInterface()
contact_planner.initialize(gait_params, height_map)
kino_dyn.initialize(gait_params)

for i in range(0:steps):
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