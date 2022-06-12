import raisimpy as raisim
import os
import numpy as np
import time

# Raisim Stuff
world = raisim.World()
world.setTimeStep(0.01)
ground = world.addGround()

server = raisim.RaisimServer(world)
server.launchServer(8080)

model_path = os.getcwd() + '../robots/atlas/urdf/atlas.urdf'
robot = world.addArticulatedSystem(model_path)
robot.setName("ex_robot")

print("Number of Generalized Coordinates: ", robot.getGeneralizedCoordinateDim())
print("Number of Controllable DOF: ", robot.getDOF())

initial_config = np.array([0.0, 0.0, 0.95,        #X, Y, Z
                        0.0, 0.0, 0.0, 1.0,     #Orientation (quaternion) in pinocchio coordinates (x,y,z,w)
                        0.0, 0.0, 0.0,     #Pelvis
                        0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

robot.setState(initial_config, np.zeros(robot.getDOF()))

time.sleep(2)
for i in range(5000000000):
    time.sleep(world.getTimeStep())

    # Update State
    robot.setState(initial_config, np.zeros(robot.getDOF()))

