from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class Rai_Headless(Node):

    def __init__(self):
        super().__init__("Raisim_headless")

        self.robot = Solo12Env(2.5, 0.05)

        self.joint_env_pos_publisher = self.create_publisher(Float64MultiArray, "estimate_joint_pos",10)
        self.joint_env_vel_publisher = self.create_publisher(Float64MultiArray, "estimate_joint_vel",10)

        timer_period = 0.0001 #seconds
        self.timer = self.create_timer(timer_period, self.simulate)

        self.mpc_joint_pos_subscriber = self.create_subscription(
            Float64MultiArray,
            'desired_joint_pos',
            self.listener_callback_position,
            10)
        self.mpc_joint_pos_subscriber

        self.mpc_joint_vel_subscriber = self.create_subscription(
            Float64MultiArray,
            'desired_joint_vel',
            self.listener_callback_velocity,
            10)
        self.mpc_joint_vel_subscriber

        self.mpc_joint_accel_subscriber = self.create_subscription(
            Float64MultiArray,
            'desired_joint_accel',
            self.listener_callback_accel,
            10)
        self.mpc_joint_accel_subscriber

        self.mpc_ee_forces = self.create_subscription(
            Float64MultiArray,
            'desired_ee_forces',
            self.listener_callback_forces,
            10)
        self.mpc_ee_forces_subscriber

        self.q_des = 19 * [0.0]
        self.dq_des = 18 * [0.0]
        self.us = 18 * [0.0]
        self.ee_forces = 12 * [0.0]

    def simulate(self):
        q, v = self.robot.get_state()
        self.robot.send_joint_command(self.q_des, self.dq_des, self.us[index], self.ee_forces[index])

        env_q = Float64MultiArray()
        env_v = Float64MultiArray()

        env_q.data = q.tolist()
        env_v.data = v.tolist()

        self.joint_env_pos_publisher_.publish(env_q)
        self.joint_env_vel_publisher_.publish(env_v)

    def listener_callback_position(self, msg):
        self.q_des = msg.data

    def listener_callback_velocity(self, msg):
        self.dq_des = msg.data

    def listener_callback_accel(self, msg):
        self.us = msg.data

    def listener_callback_forces(self, msg):
        self.ee_forces = msg.data


def main(args=None):
    rclpy.init(args=args)

    Rai_Headless_Node = Rai_Headless()

    rclpy.spin(Rai_Headless_Node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Rai_Headless_Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()