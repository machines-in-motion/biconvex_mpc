from abc import abstractmethod


class AbstractEnv:
    def __init__(self, urdf_path, robot_info, vis_ghost=False):
        self.vis_ghost = vis_ghost

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
        self.env.step()  # You can sleep here if you want to slow down the replay

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

    @abstractmethod
    def step(self):
        pass