## this file contains A1 raisim env for mpc testing

import numpy as np
from blmc_controllers.robot_id_controller import InverseDynamicsController
from raisim_utils.rai_env import RaiEnv
import time
from py_biconvex_mpc.bullet_utils.a1_state_estimator import A1StateEstimator

class A1Env:

    def __init__(self, kp, kd, q0, v0, pinModel = None, pinData= None, vis_ghost = False, loadBullet = False):

        self.kp = kp
        self.kd = kd

        #Change the urdf_path to load from raisim_utils
        urdf_path =  "/home/pshah/Applications/raisim_utils/urdf/a1/urdf/a1.urdf"
        model_path = "/home/pshah/Applications/raisim_utils/urdf/a1/urdf" #Needed for TSID

        self.vis_ghost = vis_ghost

        print("loading rai")
        self.env = RaiEnv()
        self.rai_robot = self.env.add_robot(urdf_path = urdf_path, init_config = q0, vis_ghost = self.vis_ghost)
        self.env.launch_server()

        self.f_arr = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]

        # self.robot_tsid_ctrl = TSID_controller(self.robot, urdf_path, model_path, self.f_arr, q0, v0)

        self.robot_id_ctrl = InverseDynamicsController(self.rai_robot, self.f_arr, pinModel = pinModel, pinData = pinData)
        self.robot_id_ctrl.set_gains(kp, kd, [10.0, 10.0, 10.0], [1.0, 1.0, 1.0], [200.0, 200.0, 200.0], [50.0, 50.0, 50.0])

        # state estimator
        self.state_estimator = A1StateEstimator(pinModel, pinData)

    def get_state(self):
        """
        returns the current state of the robot
        """
        q, v = self.rai_robot.get_state()
        return q, v

    def get_hip_locations(self):
        """
        returns hip locations
        """
        q, v = self.rai_robot.get_state()
        hip_loc = self.state_estimator.return_hip_locations(q, v)

        return hip_loc

    def send_joint_command(self, q_des, v_des, a_des, F_des, contact_configuration):
        """
        computes the torques using the ID controller and plugs the torques
        Input:
            q_des : desired joint configuration at current time step
            v_des : desired joint velocity at current time step
            F_des : desired feed forward forces at current time step
        """
        q, v = self.rai_robot.get_state()
        self.rai_robot.forward_robot(q,v)
        tau = self.robot_id_ctrl.id_joint_torques(q, v, q_des, v_des, a_des, F_des, contact_configuration)
        self.rai_robot.send_joint_command(tau)
        if self.vis_ghost:
            self.rai_robot.set_state_ghost(q_des, v_des)
        self.env.step() # You can sleep here if you want to slow down the replay

    # def send_joint_command_tsid(self, t, q_des, v_des, a_des, F_des, des_contact_arr):
    #     """
    #     computes the torques using the ID controller and plugs the torques
    #     Input:
    #         q_des : desired joint configuration at current time step
    #         v_des : desired joint velocity at current time step
    #         F_des : desired feed forward forces at current time step
    #     """
    #     q, v = self.robot.get_state()
    #     self.robot.forward_robot(q,v)
    #     tau = self.robot_tsid_ctrl.compute_id_torques(t, q, v, q_des, v_des, a_des, F_des, des_contact_arr)
    #     self.robot.send_joint_command(tau)
    #     if self.vis_ghost:
    #         self.robot.set_state_ghost(q_des, v_des)
    #     self.env.step() # You can sleep here if you want to slow down the replay

    def get_current_contacts(self):
        """
        :return: an array of boolean 1/0 of end-effector current status of contact (0 = no contact, 1 = contact)
        """
        contact_configuration = self.rai_robot.get_current_contacts()
        return contact_configuration
