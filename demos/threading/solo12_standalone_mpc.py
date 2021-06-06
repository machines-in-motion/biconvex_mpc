## This is a demo for trot motion in mpc
## Author : Avadesh Meduri
## Date : 21/04/2021

import numpy as np
import pinocchio as pin
#from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
#from py_biconvex_mpc.ik_utils.gait_generator import GaitGenerator
from robot_properties_solo.config import Solo12Config
from mpc_gait_gen import SoloMpcGaitGen

from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class MPC_loop(Node):

    def __init__(self):
        super().__init__("MPC_publisher")

        self.joint_desired_pos_publisher = self.create_publisher(Float64MultiArray, "desired_joint_pos",10)
        self.joint_desired_vel_publisher = self.create_publisher(Float64MultiArray, "desired_joint_vel",10)
        self.joint_desired_accel_publisher = self.create_publisher(Float64MultiArray, "desired_joint_accel",10)
        self.ee_desired_force_publisher = self.create_publisher(Float64MultiArray, "desired_ee_forces",10)

        timer_period = 0.001 #seconds
        self.timer = self.create_timer(timer_period, self.do_mpc)

        self.joint_pos_subscriber = self.create_subscription(
            Float64MultiArray,
            'Joint_Positions',
            self.listener_callback_position,
            10)
        self.joint_pos_subscriber

        self.joint_vel_subscriber = self.create_subscription(
            Float64MultiArray,
            'Joint_Velocities',
            self.listener_callback_velocity,
            10)
        self.joint_vel_subscriber

        self.joint_states = 13 * [0.0]
        self.joint_vel = 12 * [0.0]

        self.pin_robot = Solo12Config.buildRobotWrapper()
        self.urdf_path = Solo12Config.urdf_path
        st = 0.2
        self.dt = 5e-2
        state_wt = np.array([0.] * 3 + [1000.] * 3 + [5.0] * (self.pin_robot.model.nv - 6) \
                            + [0.01] * 6 + [5.0] *(self.pin_robot.model.nv - 6))

        q0 = np.array(Solo12Config.initial_configuration)
        x0 = np.concatenate([q0, pin.utils.zero(self.pin_robot.model.nv)])

        self.v_des = np.array([0.0, 0, 0])
        sl_arr = self.v_des*st
        self.t = 0.0
        self.sh = 0.15
        plan_freq = 0.05 # sec

        self.gg = SoloMpcGaitGen(self.pin_robot, self.urdf_path, st, self.dt, state_wt, x0, plan_freq, gait = 0)

        #self.robot = Solo12Env(2.5, 0.05)

    def do_mpc(self):
        next_loc = np.array([[ 0.3946 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                             [ 0.3946 + sl_arr[0],  -0.14695 + sl_arr[1], 0],
                             [ 0.0054 + sl_arr[0],   0.14695 + sl_arr[1], 0],
                             [ 0.0054 + sl_arr[0],  -0.14695 + sl_arr[1], 0]])

        #Assume Forward Prediction done...

        q = np.array(self.joint_states.copy())
        v = np.array(self.joint_vel.copy())
        xs, us, f = self.gg.optimize(q, v, np.round(step_t,3), n, next_loc, self.v_des, self.sh, 7e-3, 5e-4)

        q_des = xs[index][:self.pin_robot.model.nq].copy()
        dq_des = xs[index][self.pin_robot.model.nq:].copy()

        joint_des_pos_msg = Float64MultiArray()
        joint_des_vel_msg = Float64MultiArray()
        joint_des_accel_msg = Float64MultiArray()
        ee_des_force_msg = Float64MultiArray()

        joint_des_pos_msg.data = q_des.tolist()
        joint_des_vel_msg.data = dq_des.tolist()
        joint_des_accel_msg.data = us[index].tolist()
        ee_des_force_msg.msg = f[index].tolist()

        self.joint_desired_pos_publisher_.publish(joint_des_pos_msg)
        self.joint_desired_vel_publisher_.publish(joint_des_vel_msg)
        self.joint_desired_accel_publisher_.publish(joint_des_accel_msg)
        self.ee_desired_force_publisher_.publish(ee_des_force_msg)

    def listener_callback_position(self, msg):
        self.joint_states = msg.data

    def listener_callback_velocity(self,msg):
        self.joint_vel = msg.data


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MPC_loop()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
