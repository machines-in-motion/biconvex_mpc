from environment_interface.environment_interface import AbstractEnv
import time
import raisimpy as raisim
import numpy as np


class RaisimEnv(AbstractEnv):
    def __init__(self, urdf_path, robot_info, dt=0.001, overwrite_contact_links=True):
        # Set up Raisim
        self.dt = dt

        # Array of robots
        self.robots = []

        # Raisim World & Server Configuration
        self.world = raisim.World()
        self.world.setTimeStep(self.dt)
        self.server = raisim.RaisimServer(self.world)
        self.ground = self.world.addGround()

        # Raisim Robot Configuration
        self.robot = self.world.addArticulatedSystem(urdf_path)
        self.robot.setControlMode(raisim.ControlMode.FORCE_AND_TORQUE)
        self.num_eef = robot_info['num_eef']

        # Set up Raisim Robot Initial Configuration (assuming pinocchio-coordinates)
        self.q0 = np.array(robot_info['initial_configuration'])
        rq = self.q0
        w = rq[6]
        rq[4:7] = rq[3:6]
        rq[3] = w
        self.robot.setGeneralizedCoordinate(rq)

        # Robot End Effector Configuration/Information
        # TODO: Get these from robot_info?
        if overwrite_contact_links == True:
            self.ee_body_names = ["FL_LOWER_LEG", "FR_LOWER_LEG", "HL_LOWER_LEG", "HR_LOWER_LEG"]
            self.raisim_foot_idx = np.zeros(len(self.ee_body_names))
            for i, eef_name in enumerate(self.ee_body_names):
                self.raisim_foot_idx[i] = self.robot.getBodyIdx(eef_name)
        else:
            self.ee_body_names = ["l_foot", "r_foot"]
            self.raisim_foot_idx = np.zeros(len(self.ee_body_names))
            for i, eef_name in enumerate(self.ee_body_names):
                self.raisim_foot_idx[i] = self.robot.getBodyIdx(eef_name)

        # Initializations for Visualization
        self.visualize_array = []

        self.joint_limit = 5000.0 #TODO: Read this from robot_info

        # Launch RAISIM server
        self.launch_server()

    def launch_server(self):
        self.server.launchServer(8080)
        time.sleep(1)

    def reset_state(self, q, v):
        """
        Resets Robot to a specific configuration
        :param q: Joint Space
        :param v: Velocity Space
        :return: None
        """
        rq = np.concatenate([q, v])
        w = rq[6]
        rq[4:7] = rq[3:6]
        rq[3] = w
        self.robot.setGeneralizedCoordinate(rq)

    def get_state(self):
        """
        returns the current state of the robot
        """
        rq, rv = self.robot.getState()

        # switching the quaternion convention
        w = rq[3]
        rq[3:6] = rq[4:7]
        rq[6] = w

        return rq, rv

    def send_joint_command(self, tau):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            tau : input torque
        """
        tau = np.hstack((np.zeros(6), tau))

        #TODO: Read this from the robot_info file

        # if any(abs(i) >= self.joint_limit for i in tau):
        #     print("Torque limits violated")
        self.robot.setGeneralizedForce(tau)
        self.step()

    def get_current_contacts(self):
        """
        returns boolean array of which end-effectors are currently in contact
        """
        contact_config = np.zeros(self.num_eef)
        for contact in self.robot.getContacts():
            for i in range(len(self.raisim_foot_idx)):
                if contact.getlocalBodyIndex() == self.raisim_foot_idx[i]:
                    contact_config[i] = 1.0

        return contact_config

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
        returns contact forces at each end effector
        """
        forces = np.zeros(3 * self.num_eef)
        for contact in self.robot.getContacts():
            for i in range(self.num_eef):
                if contact.getlocalBodyIndex() == int(self.raisim_foot_idx[i]):
                    forces[3 * i:3 * (i + 1)] += contact.getContactFrame().dot(
                        contact.getImpulse()) / self.world.getTimeStep()

        return forces

    def step(self, sleep=False):
        """
       Integrates the simulation
       """
        if sleep:
            s = time.time()
            self.server.integrateWorldThreadSafe()
            e = time.time()
            dt_taken = e - s
            if dt_taken < self.dt:
                time.sleep(self.dt - dt_taken)
            else:
                print("Warning : simulation is slower than real time")
        else:
            self.server.integrateWorldThreadSafe()

    def visualize_contacts(self, vis_array, radius=0.175):
        """
        Goes through N-Dimensional array of points to visualize
        information
        Input: N x 3 Dimensional matrix of contact points
        """

        if vis_array.shape[0] > len(self.visualize_array):
            for i in range(vis_array.shape[0] - len(self.visualize_array)):
                self.visualize_array.append(
                    self.server.addVisualSphere("sphere" + str(len(self.visualize_array)), radius, 1, 0, 0, 1))

        elif vis_array.shape[0] < len(self.visualize_array):
            for i in range(len(self.visualize_array) - vis_array.shape[0]):
                self.visualize_array[-1 * (i + 1)].setPosition((0, 0, 100))

        for i in range(vis_array.shape[0]):
            self.visualize_array[i].setPosition((vis_array[i]))
