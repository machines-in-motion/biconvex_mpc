from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from raisim_utils.rai_env import RaiEnv

class AbstractEnv:

    def __init__(self, q0, v0, vis_ghost = False, loadBullet = False):

        urdf_path =  "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        model_path = "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf"

        # urdf_path =  "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        # model_path = "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf"

        self.vis_ghost = vis_ghost

        print("loading rai")
        self.env = RaiEnv()
        self.robot = self.env.add_robot(Solo12Config, urdf_path, vis_ghost = self.vis_ghost)
        self.robot.reset_state(q0, v0)
        self.env.launch_server()

        ## For data recording
        self.q_arr = []
        self.v_arr = []

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.robot.get_state()
        return q, v

    def send_joint_command(self, tau):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            tau : input torque
        """
        self.robot.send_joint_command(tau)
        self.env.step() # You can sleep here if you want to slow down the replay

    def get_current_contacts(self):
        """
        :return: an array of boolean 1/0 of end-effector current status of contact (0 = no contact, 1 = contact)
        """
        contact_configuration = self.robot.get_current_contacts()
        return contact_configuration

    def create_height_map(self, size, samples, terrain):
        height_map = self.env.create_height_map(size, samples, terrain)
        print(height_map)
        return height_map

    def create_height_map_perlin(self, raisimTerrain):
        height_map = self.env.create_height_map_perlin(raisimTerrain)
        return height_map

    def create_height_map_png(self, x_center, y_center, path_to_png, size, scale, z_offset):
        height_map = self.env.create_height_map_png(x_center, y_center, path_to_png, size, scale, z_offset)
        return height_map

    def get_ground_reaction_forces(self):
        """
        returns ground reaction forces from the simulator
        """
        forces = self.robot.get_contact_forces()
        return forces

