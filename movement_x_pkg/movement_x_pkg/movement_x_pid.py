import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Import class from training file (pidcontroller.py)
from movement_x_pkg.pidcontroller import pidcontrol


class MovementXNode(Node):

    def __init__(self):
        super().__init__('movement_x_node')

        self.target_distance = 2.0
        self.start_x = None
        self.start_y = None
        self.is_goal_reached = False

        # Instance from pidcontrol class
        self.move_pid = pidcontrol(deadzone=0.02)

        # Dynamic PID Subscriber (From teammate's logic)
        self.pid_sub = self.create_subscription(
            Float32MultiArray, '/move_pid', self.move_pid_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0

    def move_pid_callback(self, msg):
        """Dynamic tuning logic from teammate's node"""
        if len(msg.data) >= 3:
            self.move_pid.kp = msg.data[0]
            self.move_pid.ki = msg.data[1]
            self.move_pid.kd = msg.data[2]

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

    def control_loop(self):
        if self.start_x is None or self.is_goal_reached:
            return

        # Distance calculation using Euclidean Distance (X and Y)
        distance_traveled = math.sqrt(
            (self.current_x - self.start_x) ** 2
            + (self.current_y - self.start_y) ** 2
        )

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Computing linear velocity using pidcontrol.compute()
        linear_vel = self.move_pid.compute(
            self.target_distance, distance_traveled, current_time
        )

        twist = Twist()
        twist.linear.x = linear_vel
        self.cmd_vel_pub.publish(twist)

        # Goal check
        if abs(self.target_distance - distance_traveled) < self.move_pid.deadzone:
            self.get_logger().info('Goal reached successfully!')
            self.stop_robot()
            self.is_goal_reached = True

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MovementXNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()