import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_cmd_vel(self):
        twist = Twist()
        # Set linear velocity to move forward
        twist.linear.x = 5.0
        # Set angular velocity for turning left
        twist.angular.z = 1.0

        # Publish the velocity command for 10 seconds
        end_time = time.time() + 10
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()

    try:
        rover_controller.publish_cmd_vel()
    finally:
        # Destroy the node explicitly
        rover_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

