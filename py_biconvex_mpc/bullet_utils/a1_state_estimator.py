## Contains functions that determine the current location of various frames
## of A1 robot
## Author: Avadesh Meduri
## Date: 9/12/2019

import pinocchio as pin
import numpy as np


class A1StateEstimator:

    def __init__(self, pinModel, pinData):
        self.pinModel = pinModel
        self.pinData = pinData
        self.x = np.zeros(3)
        self.xd = np.zeros(3)
        self.robot_mass = sum([i.mass for i in pinModel.inertias[1:]])

    def get_frame_location(self, q, dq, frame_idx):
        """
            returns the global location of the frame
        """
        pin.framesForwardKinematics(self.pinModel, self.pinData, q)
        return np.reshape(np.array(self.pinData.oMf[frame_idx].translation), (3,))

    def get_frame_velocity(self, q, dq, frame_idx):
        """
            returns frame velocity
        """
        #self.pinModel.framesForwardKinematics(q)
        pin.framesForwardKinematics(self.pinModel, self.pinData, q)
        return np.reshape(np.array(pin.getFrameVelocity(self.pinModel, \
                                                        self.pinData, frame_idx, pin.LOCAL_WORLD_ALIGNED))[0:3], (3,))

    def return_com_location(self, q, dq):
        """
            returns com location
        """
        return np.reshape(np.array(q[0:3]), (3,))

    def return_com_velocity(self, q, dq):
        """
            returns com velocity
        """
        return np.reshape(np.array(dq[0:3]), (3,))

    def return_foot_locations(self, q, dq):
        """
            returns current foot location of the solo.
        """

        fl_foot = self.get_frame_location(q, dq, self.pinModel.getFrameId("FL_FOOT"))
        fr_foot = self.get_frame_location(q, dq, self.pinModel.getFrameId("FR_FOOT"))
        hl_foot = self.get_frame_location(q, dq, self.pinModel.getFrameId("HL_FOOT"))
        hr_foot = self.get_frame_location(q, dq, self.pinModel.getFrameId("HR_FOOT"))

        return fl_foot, fr_foot, hl_foot, hr_foot

    def return_hip_locations(self, q, dq):
        """
            returns current hip locations of solo
        """
        fl_hip = self.get_frame_location(q, dq, self.pinModel.getFrameId("FL_HFE"))
        fr_hip = self.get_frame_location(q, dq, self.pinModel.getFrameId("FR_HFE"))
        hl_hip = self.get_frame_location(q, dq, self.pinModel.getFrameId("HL_HFE"))
        hr_hip = self.get_frame_location(q, dq, self.pinModel.getFrameId("HR_HFE"))

        return fl_hip, fr_hip, hl_hip, hr_hip

    def return_hip_velocities(self, q, dq):
        """
            returns current hip velocities of solo
        """
        fl_hip = self.get_frame_velocity(q, dq, self.pinModel.getFrameId("FL_HFE"))
        fr_hip = self.get_frame_velocity(q, dq, self.pinModel.getFrameId("FR_HFE"))
        hl_hip = self.get_frame_velocity(q, dq, self.pinModel.getFrameId("HL_HFE"))
        hr_hip = self.get_frame_velocity(q, dq, self.pinModel.getFrameId("HR_HFE"))

        return fl_hip, fr_hip, hl_hip, hr_hip

    def return_dcm_location(self, q, dq, omega):
        """
            returns current divergent component of motion location of solo
        """
        self.x = np.reshape(np.array(q[0:3]), (3,))
        self.xd = np.reshape(np.array(dq[0:3]), (3,))
        return self.x + self.xd/omega

    def return_hip_offset(self, q, dq):
        """
            Returns the distance between the hip and the center of mass
        """
        fl_hip, fr_hip, hl_hip, hr_hip = self.return_hip_locations(q, dq)
        com = np.reshape(np.array(q[0:3]), (3,))
        return np.subtract(fl_hip, com), np.subtract(fr_hip, com), np.subtract(hl_hip, com), np.subtract(hr_hip, com)

    def return_zmp_location(self, q, dq, cnt_array):
        """
            returns the current location of the zmp
        """
        foot_loc = self.return_foot_locations(q, dq) ## computing current location of the feet
        zmp = np.zeros(2)
        for i in range(len(cnt_array)):
            if cnt_array[i] == 0:
                continue
            zmp = np.add(zmp, foot_loc[i][0:2])

        zmp = np.divide(zmp, sum(cnt_array))

        return zmp
