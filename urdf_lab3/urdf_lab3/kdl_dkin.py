#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
# https://ubuntu.pkgs.org/20.04/ubuntu-universe-arm64/python3-pykdl_1.4.0-7build2_arm64.deb.html
from PyKDL import *
import yaml
import os


class KdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('kdl_dkin')
       
        self.pose_stamped = PoseStamped()
        self.arm1connect = 0.0
        self.arm2connect = 0.0
        self.wristconnect = 0.0


        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_kdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)


        #self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # rate
        self.rate = self.create_rate(10)

    def listener_callback(self, msg):
        # get joint state
        [self.arm1connect, self.arm2connect, self.wristconnect] = msg.position

        self.set_pose_stamped()

        print(self.pose_stamped.pose.position.x)
        print(self.pose_stamped.pose.position.y)
        print(self.pose_stamped.pose.position.z)
        print(self.pose_stamped.pose.orientation)

        # publish pose of gripper
        now = self.get_clock().now()
        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'base'
        self.pose_pub.publish(self.pose_stamped)
        # self.rate.sleep()

    def set_pose_stamped(self):
        position, quaternion = self.solve_gripper_params()
        
        self.pose_stamped.pose.position.x = float(position[0])
        self.pose_stamped.pose.position.y = float(position[1])
        self.pose_stamped.pose.position.z = float(position[2])

        x,y,z,w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        self.pose_stamped.pose.orientation = Quaternion(x = float(x), y= float(y), z= float(z), w = float(w))

    def solve_gripper_params(self):
        # solving forward kinematics
        result = self.solve_kdl(self.get_chain())

        # converting to quaternion
        # gripper = self.get_frame("gripper")
        gripper = Frame()
        cart = result.JntToCart(self.get_joint_array(), gripper)
        quaternion = gripper.M.GetQuaternion()
        position = gripper.p

        return position, quaternion


    def solve_kdl(self, chain):
        return ChainFkSolverPos_recursive(chain)

    def get_joint_array(self):
        # joint_array = JntArray(5)
        joint_array = JntArray(3)

        # joint_array[0] = 0.0
        # joint_array[1] = self.arm1connect
        # joint_array[2] = self.arm2connect
        # joint_array[3] = self.wristconnect
        # joint_array[4] = 0.0 # nie mam pojecia czy to ma sens

        joint_array[0] = self.arm1connect
        joint_array[1] = self.arm2connect
        joint_array[2] = self.wristconnect

        return joint_array

    def get_chain(self):
        chain = Chain()

        # base = Segment(Joint(), self.get_frame('base'))
        arm1 = Segment(Joint(Joint.RotZ), self.get_frame('arm1'))
        arm2 = Segment(Joint(Joint.RotZ), self.get_frame('arm2'))
        wrist = Segment(Joint(Joint.TransZ), self.get_frame('wrist'))
        # gripper = Segment(Joint(), self.get_frame('gripper'))

        # chain.addSegment(base)
        chain.addSegment(arm1)
        chain.addSegment(arm2)
        chain.addSegment(wrist)
        # chain.addSegment(gripper)
    
        return chain

    def get_frame(self, name):
        rpy = self.read_yaml(os.path.dirname(os.path.realpath(__file__)) + '/dh.yaml')

        r = rpy[name]['rpy']['r']
        p = rpy[name]['rpy']['p']
        y = rpy[name]['rpy']['y']
        x = rpy[name]['xyz']['x']
        y = rpy[name]['xyz']['y']
        z = rpy[name]['xyz']['z']

        return Frame(Rotation.RPY(r,p,y), Vector(x,y,z))

    def read_yaml(self, filename):
        with open(filename, 'r') as file:
            return yaml.load(file)


def main():
    node = KdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()