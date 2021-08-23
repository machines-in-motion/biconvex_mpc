## this file contains atlas raisim env for mpc testing
## Author : Avadesh Meduri
## Date : 6/08/2021


import numpy as np

from blmc_controllers.robot_id_controller import InverseDynamicsController
from robot_properties_atlas.config import AtlasConfig

from raisim_utils.rai_env import RaiEnv

import os
import time

class AtlasEnv:

    def __init__(self, kp, kd, q0, v0, vis_ghost = False, loadBullet = False):

        self.kp = kp
        self.kd = kd

        self.vis_ghost = vis_ghost
        self.bullet = loadBullet


        if self.bullet:
            print("Not ready yet. Please use Raisim")

        else:
            print("loading rai")
            self.env = RaiEnv()
            self.robot = self.env.add_robot(AtlasConfig, AtlasConfig.urdf_path, vis_ghost = self.vis_ghost)
            print("done..")
            self.robot.reset_state(q0, v0)
            self.env.launch_server()

        self.f_arr = ["l_foot", "r_foot"]

        self.robot_id_ctrl = InverseDynamicsController(self.robot, self.f_arr)
        self.robot_id_ctrl.set_gains(kp, kd, [10.0, 10.0, 10.0], [1.0, 1.0, 1.0], [200.0, 200.0, 200.0], [50.0, 50.0, 50.0])

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.robot.get_state()
        return q, v

    def send_joint_command(self, q_des, v_des, a_des, F_des, contact_configuration):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            q_des : desired joint configuration at current time step
            v_des : desired joint velocity at current time step
            F_des : desired feed forward forces at current time step
        """
        q, v = self.robot.get_state()
        self.robot.forward_robot(q,v)
        tau = self.robot_id_ctrl.id_joint_torques(q, v, q_des, v_des, a_des, F_des, contact_configuration)
        self.robot.send_joint_command(tau)
        if self.vis_ghost:
            self.robot.set_state_ghost(q_des, v_des)
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