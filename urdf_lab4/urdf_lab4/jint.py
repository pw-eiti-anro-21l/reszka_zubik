import sys

from interpolation.srv import Interpolation
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Interpolation, 'interpolation')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Interpolation.Request()

    def send_request(self):
        # #from tutorial srv cli
        # self.req.a = int(sys.argv[1])
        # self.req.b = int(sys.argv[2])
        # self.future = self.cli.call_async(self.req)
        try:
            if(float(sys.argv[1])>0 or float(sys.argv[1]) < -1):
                self.get_logger().info('Niepoprawna wartośc przesunięcia dla członu 1') 
                raise ValueError()
            else:
                self.req.joint1_goal = float(sys.argv[1])

            if(float(sys.argv[2])>0 or float(sys.argv[2]) < -3):
                self.get_logger().info('Niepoprawna wartośc przesunięcia dla członu 2')                
                raise ValueError()
            else:
                self.req.joint2_goal= float(sys.argv[2])

            if(float(sys.argv[3])>0 or float(sys.argv[3]) < -2):
                self.get_logger().info('Niepoprawna wartośc przesunięcia dla członu 3')
                raise ValueError()
            else:
                self.req.joint3_goal = float(sys.argv[3])

            if(float(sys.argv[4])<=0):
                self.get_logger().info('Niepoprawna wartość czasu')
                raise ValueError()
            else:
                self.req.time_of_move = float(sys.argv[4])

            if(str(sys.argv[5]) !='linear' and str(sys.argv[5]) !='polynomial' ):
                self.get_logger().info('Zły typ interpolacji ')
                raise ValueError()
            else:
                self.req.type = (sys.argv[5])  
        except IndexError:
            print("Niepoprawna liczba parametrów")
            raise Exception()
        except ValueError:
            print("Błędne parametry")
            raise Exception()
            
        self.future = self.cli.call_async(self.req)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of interpolation: joint1:%d, joint2:%d, joint3:%d, time:%d = %s' %
                    (minimal_client.req.arm1_pos, minimal_client.req.arm2_pos, minimal_client.req.wrist_pos, minimal_client.req.time, response.output))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()