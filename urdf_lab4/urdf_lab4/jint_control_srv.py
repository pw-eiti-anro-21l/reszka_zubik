import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from interpolation.srv import Interpolation


class JintControlService(Node):

    def __init__(self):
        super().__init__('jint_service')
        self.service = self.create_service(Interpolation, 'interpolation', interpolation_callback)


    def interpolation_callback(self, msg, output):

        time = msg.time
        freq = 0.1

        if msg.type is "linear":
            pass
        
        else:
            pass
            # inne metody lub error ze niepoprawne dane


if __name__ == '__main__':

    rclpy.init(args=args)

    jint_control_service = JintControlService()

    rclpy.spin(jint_control_service)

    rclpy.shutdown()