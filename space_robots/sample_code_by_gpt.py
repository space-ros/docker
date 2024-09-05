import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class DifferentialRoverNode(Node):
    def __init__(self):
        super().__init__('differential_rover_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        self.movement_duration = 20  # Move for 20 seconds
        self.start_time = self.get_clock().now().to_msg().sec

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg().sec
        if current_time - self.start_time < self.movement_duration:
            move_cmd = Twist()
            move_cmd.linear.x = 10.0  # Move forward at 10m/s
            move_cmd.angular.z = 0.0  # No rotation
            self.publisher.publish(move_cmd)
        else:
            self.stop_movement()

    def stop_movement(self):
        move_cmd = Twist()  # Create a new message to stop the robot
        self.publisher.publish(move_cmd)
        self.get_logger().info('Stopping')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialRoverNode()
    rclpy.spin(node)
    rclpy.shutdown()

