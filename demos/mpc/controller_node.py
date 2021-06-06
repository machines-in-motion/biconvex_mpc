import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

from robot_properties_solo.config import Solo12Config
from mpc_gait_gen import SoloMpcGaitGen
from py_biconvex_mpc.bullet_utils.solo_mpc_env import Solo12Env

#from whole_body_state_msgs.msg import JointState

import numpy as np

import pinocchio as pin

class SimControlNode(Node):

    def __init__(self):
        super().__init__('Raisim_Controller_Node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.real_time_rate = 0.001 #How often the callback to the "controller + simulator" setup should be called
        self.sim_dt = 0.001 #Simulation dt
        self.timer = self.create_timer(self.real_time_rate, self.call_controller)
        self.time = 0.0

        #Setup controller and raisim environment
        self.pin_robot = Solo12Config.buildRobotWrapper()
        self.urdf_path = Solo12Config.urdf_path
        q0 = np.array(Solo12Config.initial_configuration)
        v0 = pin.utils.zero(self.pin_robot.model.nv)

        self.robot = Solo12Env(2.15, 0.03, q0, v0)

        #Create subscribers that you will get from the optimizer
        self.state_subscriber =  self.create_subscription(
            Float64MultiArray,
            'q_des',
            self.listener_callback,
            10)
        self.state_subscriber

        self.pos_subscriber = self.create_subscription(
            Float64MultiArray,
            'q_des',
            self.listener_callback,
            10)
        self.pos_subscriber  # prevent unused variable warning
        self.q_des = self.pin_robot.model.nq*2 * [0]

        self.vel_subscriber = self.create_subscription(
            Float64MultiArray,
            'v_des',
            self.listener_callback,
            10)
        self.vel_subscriber  # prevent unused variable warning
        self.v_des = self.pin_robot.model.nv*2 * [0]

        self.accel_subscriber = self.create_subscription(
            Float64MultiArray,
            'a_des',
            self.listener_callback,
            10)
        self.accel_subscriber  # prevent unused variable warning
        self.a_des = self.pin_robot.model.nv*2 * [0]

        self.force_subscriber = self.create_subscription(
            Float64MultiArray,
            'a_des',
            self.listener_callback,
            10)
        self.force_subscriber  # prevent unused variable warning
        self.f_des = self.num_eef*3*2 * [0]

    def call_controller(self):

        #Send command to robot and integrate environment/world
        self.robot.send_joint_command(self.q_des, self.v_des, self.a_des[index], self.f_des[index])

        #Get new state and put into ROS2 publisher
        state_and_time = Float64MultiArray()
        q,v = self.robot.get_state()

        #Publish State
        self.publisher_.publish(state_and_time)

        #Increment time
        self.time += 0.001

    def listener_callback(self, msg):
        self.


def main(args=None):
    rclpy.init(args=args)

    sim_publisher = SimControlNode()

    rclpy.spin(sim_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()