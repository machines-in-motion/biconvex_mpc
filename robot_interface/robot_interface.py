#This is a robot interface class to get states, interact with a virtual world, etc.

import raisimpy as raisim

class RobotInterface:

    def __init__(self, urdf, dt, isRealRobot = False):
        """
        Input:
            r_urdf : urdf of robot
            dt : discretization
        """
        self.dt = dt
        if isRealRobot():
            
        else:
            # Set up Raisim World
            self.world = raisim.World()
            self.world.setTimeStep(self.dt)
            self.server = raisim.RaisimServer(self.world)
            self.ground = self.world.addGround()

            # Set up Raisim Robot
            self.robot = self.world.addArticulatedSystem(urdf_path)
            self.robot.setName("cool_robot")
            self.robot.setControlMode(raisim.ControlMode.FORCE_AND_TORQUE)
    
    def create_height_map(self):
        """
        Creates height map
        """
        
    def get_contact_forces(self):
        """
        returns contact forces for each end-effectors
        """

    def get_current_contacts(self):
        """
        returns boolean array of which end-effectors are in contact
        """

    def get_ee_positions(self):

    def get_state():
        rq, rv = self.robot.getState()

        # Switching Quaternion Convention due to raisim/pinocchio convention
        w = rq[3]
        rq[3:6] = rq[4:7]
        rq[6] = w

        return rq, rv

    def send_joint_command(self, tau):
        if not isRealRobot:
            self.robot.setGeneralizedForce(tau)