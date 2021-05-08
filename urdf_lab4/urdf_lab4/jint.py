import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from math import floor
from interpolation.srv import JintControl
from sensor_msgs.msg import JointState
import time


class JintControlService(Node):

    def __init__(self):
        super().__init__('jint_service')

        self.service = self.create_service(JintControl, 'jint_control', self.jint_control_callback)
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        self.joint_state = JointState()
        self.joint_state.name = ['arm1connect', 'arm2connect', 'wristconnect']
        self.arm1connect = 0
        self.arm2connect = 0
        self.wristconnect = 0
        self.time = 0

    def jint_control_callback(self, msg, response):

        sample_time = msg.sample_time

        while(self.time < msg.time):

            if msg.time <= 0:
                return "Error: forbiden interpolation time"

            # elif: inne niepoprawne parametry

            elif msg.type == "linear":
                arm1connect = self.interpolate(msg.time, msg.arm1_pos)
                arm2connect = self.interpolate(msg.time, msg.arm2_pos)
                wristconnect = self.interpolate(msg.time, msg.wrist_pos)
                self.joint_state.position = [arm1connect, arm2connect, wristconnect]
            
            else:
                pass
                # inne metody

            self.get_logger().info(f"arm1: {arm1connect}, arm2: {arm2connect}, wrist: {wristconnect}, time: {self.time}")

            self.joint_pub.publish(self.joint_state)

            self.time += sample_time

            time.sleep(sample_time)


        response.output = 'Interpolation completed'
        return response
        

    def interpolate(self,time, end_point): # TODO zmienic zeby dzialalo dla roznych pkt pocz.
        # (x0,t0) = (0,0)

        x1 = end_point
        t1 = time

        return (x1/t1)*self.time


def main(args=None):

    rclpy.init(args=args)

    jint_control_service = JintControlService()

    rclpy.spin(jint_control_service)

    rclpy.shutdown()


if __name__ == '__main__':

    main()