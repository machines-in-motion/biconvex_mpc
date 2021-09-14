#This is a robot interface class to get states, interact with a virtual world, etc.

import raisimpy as raisim
import pinocchio as pin

class RobotInterface:

    def __init__(self, urdf, dt, robot_model, isRealRobot = False):
        """
        Input:
            r_urdf : urdf of robot
            dt : discretization
        """
        self.dt = dt

        # Set up Pinocchio stuff (Is this necessary...?)
        self.pin_model = pin.buildModelFromUrdf(urdf, 
                                                pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()

        if isRealRobot():
            
        else:
            # Set up Raisim World
            self.world = raisim.World()
            self.world.setTimeStep(self.dt)
            self.server = raisim.RaisimServer(self.world)
            self.ground = self.world.addGround()

            # Set up Raisim Robot
            self.robot = self.world.addArticulatedSystem(urdf)
            self.robot.setName("cool_robot")
            self.robot.setControlMode(raisim.ControlMode.FORCE_AND_TORQUE)

            # Switching quaternion convention due to raisim/pinocchio differences
            rq = gait_params.init_config
            w = rq[6]
            rq[4:7] = rq[3:6]
            rq[3] = w
            self.robot.setGeneralizedCoordinates(rq)
    
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

    def get_state():
        """
        Get robot state (q, v)
        """

        if not isRealRobot:
            rq, rv = self.robot.getState()
            # Switching quaternion convention due to raisim/pinocchio differences
            w = rq[3]
            rq[3:6] = rq[4:7]
            rq[6] = w

        pin.forwardKinematics(self.pin_model, self.pin_data, rq)
        pin.framesForwardKinematics(self.pin_model, self.pin_data, rq)
        pin.computeJointJacobians(self.pin_model, self.pin_data, rq)
        pin.computeCentroidalMomentum(self.pin_model, self.pin_data, rq, rv)

        return rq, rv

    def send_joint_command(self, tau):
        if not isRealRobot:
            self.robot.setGeneralizedForce(tau)

    def visualize(self, contact_locations):
