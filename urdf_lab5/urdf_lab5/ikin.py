from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class Ikin(Node):

  def __init__(self):

      rclpy.init()
      super().__init__('ikin')
      self.values = readDHfile()

      self.subscription = self.create_subscription(PoseStamped(),'/inv_pose_interpolation',self.listener_callback,10)
      self.subscription

      loop_rate = self.create_rate(30)

      joint_state = JointState()

  def listener_callback(self,msg):


      ##powinno być pose.pose albo msg.pose
      pose = PoseStamped()
      arm1_connect = pose.pose.position.x
      arm2_connect = pose.pose.position.y
      wrist_connect = pose.pose.position.z


      ##kinematyka odwrotna rys

      #publikowanie
      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))
      
      joint_state = JointState()
      now = self.get_clock().now()
      joint_state.header.stamp = now.to_msg()
      joint_state.name = ['arm1_connect', 'arm2_connect', 'wrist_connect']



      self.joint_pub.publish(joint_state)


def readDHfile():

    with open(os.path.join(get_package_share_directory('urdf_lab5'),'dh.json'), 'r') as file:

        content = json.loads(file.read())

    return content

def main():
  node = Ikin()

if __name__ == '__main__':
  main()
