## this file contains solo raisim env for mpc testing
## Author : Avadesh Meduri
## Date : 7/05/2021


import numpy as np

from blmc_controllers.robot_id_controller import InverseDynamicsController
from raisim_utils.rai_env import RaiEnv
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

class Solo12Env:

    def __init__(self, kp, kd):

        self.kp = kp
        self.kd = kd

        self.env = RaiEnv()
        #Change the urdf_path to load from raisim_utils
        urdf_path =  "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        self.robot = self.env.add_robot(Solo12Config, urdf_path)
        self.env.launch_server()

        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

        self.robot_id_ctrl = InverseDynamicsController(self.robot, self.f_arr)
        self.robot_id_ctrl.set_gains(kp, kd)

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.robot.get_state()
        return q, v

    def send_joint_command(self, q_des, v_des, a_des, F_des):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            q_des : desired joint configuration at current time step
            v_des : desired joint velocity at current time step
            F_des : desired feed forward forces at current time step
        """
        q, v = self.robot.get_state()
        tau = self.robot_id_ctrl.id_joint_torques(q, v, q_des, v_des, a_des, F_des)
        self.robot.send_joint_command(tau)
        self.env.step() # You can sleep here if you want to slow down the replay
