## this file contains a simulation env with pybullet
## Author : Avadesh Meduri
## Date : 7/05/2021

import pybullet
from bullet_utils.env import BulletEnvWithGround

class PyBulletEnv:

    def __init__(self, robot, q0, v0):

        print("loading bullet")
        self.env = BulletEnvWithGround()
        self.robot = self.env.add_robot(robot())
        self.robot.reset_state(q0, v0)
        # pybullet.resetDebugVisualizerCamera( cameraDistance=-1.5, cameraYaw=45, cameraPitch=-10, cameraTargetPosition=[0,0,.8])
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
        contact_configuration = self.robot.get_force()[0]
        return contact_configuration

    def get_ground_reaction_forces(self):
        """
        returns ground reaction forces from the simulator
        """
        forces = self.robot.get_contact_forces()
        return forces

    def start_recording(self, file_name):
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_recording(self):
        pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)
