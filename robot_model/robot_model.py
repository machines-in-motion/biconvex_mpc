#This is a robot model class to get and set 
import pinocchio as pin

class RobotModel:

    def __init__(self, urdf):
        """
        Input:
            urdf : path of urdf of robot
        """

        # Set up Pinocchio stuff (Is this necessary...?)
        self.pin_model = pin.buildModelFromUrdf(urdf, 
                                                pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()

        self.initial_configuration =  (
            [0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
            + 2 * [0.0, 0.8, -1.6]
            + 2 * [0.0, -0.8, 1.6]
            )
        self.offsets = 
        self.I_comp_b = 
        self.

    def get_ee_positions(self):