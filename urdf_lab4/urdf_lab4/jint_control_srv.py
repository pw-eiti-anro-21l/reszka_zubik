from interpolation.srv import Interpolation

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Interpolation, 'interpolation', self.interpolation_callback)

    def interpolation_callback(self, request, response):
        # #tutorial srv cli
        # response.sum = request.a + request.b
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        # Pozycje początkowe w stawach
        start_positions = [0, 0, 0]
                                               
        self.get_logger().info('Incoming request')


        sample_time = 0.1 # przykładowo co 0.1s wysyłamy wiadomość
        total_time = request.time_of_move
        steps = math.floor(total_time/sample_time) # całkowita liczba kroków do pętli

        # Wyznaczenie współczynników dla metody wielomianowej
        if(request.type == 'polynomial'):
            a0 = [start_positions[0], start_positions[1], start_positions[2]]
            a2 = [3*((request.joint1_goal - start_positions[0])/(request.time_of_move)**2),
            3*((request.joint2_goal - start_positions[1])/(request.time_of_move)**2),
            3*((request.joint3_goal - start_positions[2])/(request.time_of_move)**2)]
            a3 = [-2*((request.joint1_goal - start_positions[0])/(request.time_of_move)**3),
            -2*((request.joint2_goal - start_positions[1])/(request.time_of_move)**3),
            -2*((request.joint3_goal - start_positions[2])/(request.time_of_move)**3),]

        for i in range(1,steps+1):
            
            # Publisher dla Kartmana
            qos_profile1 = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile1)
            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['arm1connect', 'arm2connect', 'wristconnect']

            # Interpolacja liniowa
            if(request.type == 'linear'):
                joint1_next = start_positions[0] + ((request.joint1_goal - start_positions[0])/request.time_of_move)*sample_time*i
                joint2_next = start_positions[1] + ((request.joint2_goal - start_positions[1])/request.time_of_move)*sample_time*i
                joint3_next = start_positions[2] + ((request.joint3_goal - start_positions[2])/request.time_of_move)*sample_time*i

            # interpolacja wielomianowa
            if(request.type == 'polynomial'):
                joint1_next = a0[0] + a2[0]*(sample_time*i)**2 + a3[0]*(sample_time*i)**3 
                joint2_next = a0[1] + a2[1]*(sample_time*i)**2 + a3[1]*(sample_time*i)**3 
                joint3_next = a0[2] + a2[2]*(sample_time*i)**2 + a3[2]*(sample_time*i)**3 

            #Przypisanie wartości dla członów robota
            joint_state.position = [float(joint1_next), float(joint2_next), float(joint3_next)]

            # Publikowanie połozenia człónów robota
            self.joint_pub.publish(joint_state)
            time.sleep(sample_time)



        response.confirmation = 'Interpolacja zakończona'

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()