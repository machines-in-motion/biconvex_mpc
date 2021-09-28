#This is a robot model class to get and set 
import pinocchio as pin
import numpy as np

class RobotModel:

    def __init__(self, urdf, robot_info):
        """
        Input:
            urdf : path of urdf of robot
        """

        # Set up Pinocchio
        self.pin_model = pin.buildModelFromUrdf(urdf, 
                                                pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()

        #Offsets which are used in contact planner
        self.offsets = robot_info.offsets

        self.initial_configuration =  robot_info.initial_configuration

        self.pin.forwardKinematics(self.pin_model, self.pin_data, self.initial_configuration)
        self.pin.framesForwardKinematics(self.pin_model, self.pin_data, self.initial_configuration)
        self.pin.computeJointJacobians(self.pin_model, self.pin_data, self.initial_configuration)
        self.pin.computeCentroidalMomentum(self.pin_model, self.pin_data, \
            self.initial_configuration, np.zeros_like(self.initial_configuration))

        #Composite inertia of entire robot
        self.pin.crba(self.rmodel, self.rdata, q0)
        self.I_comp_b = self.pin_data.Ycrb[1].inertia