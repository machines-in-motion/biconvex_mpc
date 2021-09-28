## Demo file for running pure trajectory optimization


simulation_steps = 1091234982374982
sim_dt = 0.0
dt = 0.001

#Initialize Classes
robot_interface = RobotInterface(robot_model)
contact_planner.initialize(gait_params, height_map)
kino_dyn.initialize(gait_params)

#Compute Trajectory once
q, v = robot_interface.get_state()
contact_planner.create_contacts(q,v)
q_des, qd_des, qdd_des = kino_dyn.compute_trajectory(sim_dt, q, v, contact_plan)

for i in range(0:steps):
	sim_dt += dt

	#Get State
	q, v = robot_interface.get_state()

	#Visualize Anything we might want
	robot_interface.visualize(contact_plan)

	#Compute torques from controller
	tau = controller.compute_torque(q, v. q_des, qd_des, qdd_des)

	#Send Torque to robot/simulation
	robot_interface.send_torque(tau)