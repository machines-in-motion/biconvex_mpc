#This is a robot interface class to get states, interact with a virtual world, etc.
import pinocchio as pin

class AbstractInterface:

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

        # Is this a real robot
        self.isRealRobot = isRealRobot
    
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

    def get_state(self):
        """
        Get robot state (q, v)
        """

        if not self.isRealRobot:
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
        """
        Sends joint torques to robot and integrates
        """
        if not self.isRealRobot:
            self.robot.setGeneralizedForce(tau)
            self.server.integrateWorldThreadSafe()

    def visualize(self, contact_locations):
        """
        Visualize contact locations
        """
        return None
