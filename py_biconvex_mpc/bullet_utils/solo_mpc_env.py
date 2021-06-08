## this file contains solo raisim env for mpc testing
## Author : Avadesh Meduri
## Date : 7/05/2021


import numpy as np

from blmc_controllers.robot_id_controller import InverseDynamicsController
from blmc_controllers.tsid_controller import TSID_controller
from raisim_utils.rai_env import RaiEnv
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from py_biconvex_mpc.bullet_utils.solo_state_estimator import SoloStateEstimator


class Solo12Env:

    def __init__(self, kp, kd, q0, v0, vis_ghost = False):

        self.kp = kp
        self.kd = kd

        self.env = RaiEnv()
        #Change the urdf_path to load from raisim_utils
        urdf_path =  "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        model_path = "/home/pshah/Applications/raisim_utils/urdf/solo12/urdf"

        #urdf_path =  "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf/solo12.urdf"
        #model_path = "/home/ameduri/devel/workspace/robot_properties/raisim_utils/urdf/solo12/urdf"

        self.vis_ghost = vis_ghost
        self.robot = self.env.add_robot(Solo12Config, urdf_path, vis_ghost = self.vis_ghost)
        self.env.launch_server()

        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

        self.robot_tsid_ctrl = TSID_controller(self.robot, urdf_path, model_path, self.f_arr, q0, v0)

        self.robot_id_ctrl = InverseDynamicsController(self.robot, self.f_arr)
        self.robot_id_ctrl.set_gains(kp, kd, [10.0, 10.0, 10.0], [1.0, 1.0, 1.0], [200.0, 200.0, 200.0], [50.0, 50.0, 50.0])

        # state estimator
        self.sse = SoloStateEstimator(self.robot.pin_robot)

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.robot.get_state()
        return q, v

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

    def send_joint_command_tsid(self, t, q_des, v_des, a_des, F_des, des_contact_arr):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            q_des : desired joint configuration at current time step
            v_des : desired joint velocity at current time step
            F_des : desired feed forward forces at current time step
        """
        q, v = self.robot.get_state()
        self.robot.forward_robot(q,v)
        tau = self.robot_tsid_ctrl.compute_id_torques(t, q, v, q_des, v_des, a_des, F_des, des_contact_arr)
        self.robot.send_joint_command(tau)
        if self.vis_ghost:
            self.robot.set_state_ghost(q_des, v_des)
        self.env.step() # You can sleep here if you want to slow down the replay

    def get_current_contacts(self):
        """
        :return: an array of boolean 1/0 of end-effector current status of contact (0 = no contact, 1 = contact)
        """
        contact_configuration = self.robot.get_current_contacts()
        return contact_configuration
