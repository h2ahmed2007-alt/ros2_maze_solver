import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_actions.action import Movementros2


class MovementXActionServer(Node):

    def __init__(self):
        super().__init__('movement_x_action_server')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_x = 0.0
        self.current_y= 0.0

        self._action_server = ActionServer(self, Movementros2, 'move_robot_x',self.execute_callback)


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        

    def execute_callback(self, goal_handle):
        target_distance = goal_handle.request.distance
        start_x = self.current_x
        start_y = self.current_y

        twist = Twist()
        twist.linear.x = 0.2
        rate=self.create_rate(10)

        while rclpy.ok():
            distance_traveled = math.sqrt ((self.current_x - start_x) **2 + (self.current_y - start_x) **2)

            feedback_msg = Movementros2.Feedback()
            feedback_msg.current_distance = float(distance_traveled)
            goal_handle.publish_feedback(feedback_msg)

            if distance_traveled >= target_distance:
                self.stop_robot()
                goal_handle.succeed()

                result = Movementros2.Result()
                result.success = True
                return result

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    action_server = MovementXActionServer()
    from rclpy.executors import MultiThreadedExecutor
    executor =MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        action_server.destroy_node()    
        rclpy.shutdown()


if __name__ == '__main__':
    main()