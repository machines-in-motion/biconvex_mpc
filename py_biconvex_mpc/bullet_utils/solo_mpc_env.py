## this file contains solo raisim env for mpc testing
## Author : Avadesh Meduri
## Date : 7/05/2021


import numpy as np

from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

from raisim_utils.rai_env import RaiEnv
from py_biconvex_mpc.bullet_utils.solo_state_estimator import SoloStateEstimator

import os
import time

class Solo12Env:

    def __init__(self, q0, v0, vis_ghost = False, loadBullet = False):

        #Change the urdf_path to load from raisim_utils
        str = os.path.dirname(os.path.abspath(__file__))
        str_2 = str.replace("py_biconvex_mpc/bullet_utils", "")
        str_3 = str_2 + "robots/urdf/solo12/solo12.urdf"

        # urdf_path =  "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        # model_path = "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf"

        urdf_path =  "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        model_path = "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf"
        
        self.vis_ghost = vis_ghost
        self.bullet = loadBullet

        if self.bullet:
            print("loading bullet")
            self.env = BulletEnvWithGround()
            self.robot = self.env.add_robot(Solo12Robot)
            self.robot.reset_state(q0, v0)

        else:
            print("loading rai")
            self.env = RaiEnv()
            self.robot = self.env.add_robot(Solo12Config, urdf_path, vis_ghost = self.vis_ghost)
            self.robot.reset_state(q0, v0)
            self.env.launch_server()

        # state estimator
        self.sse = SoloStateEstimator(self.robot.pin_robot)

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.robot.get_state()
        return q, v

    def get_com_location(self):
        """
        returns com locations
        """
        q, v = self.robot.get_state()
        return self.sse.return_com_location(q, v)

    def get_hip_locations(self):
        """
        returns hip locations
        """
        q, v = self.robot.get_state()
        hip_loc = self.sse.return_hip_locations(q, v)

        hip_loc[0][1] += 0.05        
        hip_loc[1][1] -= 0.05
        hip_loc[2][1] += 0.05
        hip_loc[3][1] -= 0.05

        return hip_loc

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
        if self.bullet:
            contact_configuration = self.robot.get_force()[0]
        else:
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
