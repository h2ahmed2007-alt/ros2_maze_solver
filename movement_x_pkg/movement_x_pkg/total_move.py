import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from robot_actions.action import YawRobot

from movement_x_pkg.pid_controller import pidcontrol


class RobotMoverNode(Node):

    def __init__(self):
        super().__init__('robot_mover_node')

        self.cb_group = ReentrantCallbackGroup()

        # ---------- Linear Movement Variables ----------
        self.target_distance = 7.0
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.is_linear_goal_reached = False
        self.is_rotating = False

        self.move_pid = pidcontrol(deadzone=0.02)

        # ---------- Angular Movement Variables (Yaw Action) ----------
        self.current_yaw = 0.0
        self.last_odom_time = None

        self.yaw_pid = pidcontrol(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            min_out=-1.5,
            max_out=1.5,
            i_max=1.0,
            deadzone=0.02,
            is_angle=True,
        )

        # ---------- Publishers & Subscribers ----------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.move_pid_sub = self.create_subscription(
            Float32MultiArray,
            '/move_pid',
            self.move_pid_callback,
            10,
            callback_group=self.cb_group,
        )

        self.yaw_pid_sub = self.create_subscription(
            Float32MultiArray,
            '/yaw_pid',
            self.yaw_pid_callback,
            10,
            callback_group=self.cb_group,
        )

        # ---------- Timers & Action Server ----------
        self.linear_timer = self.create_timer(
            0.05, self.linear_control_loop, callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self,
            YawRobot,
            'yawRobot',
            execute_callback=self.execute_yaw_callback,
            callback_group=self.cb_group,
        )

    # ------------------ Callback Functions ------------------

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

        q = msg.pose.pose.orientation
        self.current_yaw = self.angle_of_orientation(q)
        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def angle_of_orientation(q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def move_pid_callback(self, msg):
        if len(msg.data) >= 3:
            self.move_pid.kp = msg.data[0]
            self.move_pid.ki = msg.data[1]
            self.move_pid.kd = msg.data[2]
            self.get_logger().info(
                f'Updated Linear PID: Kp={self.move_pid.kp}, Ki={self.move_pid.ki}, Kd={self.move_pid.kd}'
            )

    def yaw_pid_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn('Received malformed /yaw_pid message, ignoring.')
            return

        self.yaw_pid.kp = msg.data[0]
        self.yaw_pid.ki = msg.data[1]
        self.yaw_pid.kd = msg.data[2]
        self.get_logger().info(
            f'Updated Yaw PID: Kp={self.yaw_pid.kp}, Ki={self.yaw_pid.ki}, Kd={self.yaw_pid.kd}'
        )

    # ------------------ Control Loops ------------------

    def linear_control_loop(self):
        if self.start_x is None or self.is_linear_goal_reached or self.is_rotating:
            return

        distance_traveled = math.sqrt(
            (self.current_x - self.start_x) ** 2
            + (self.current_y - self.start_y) ** 2
        )
        current_time = self.get_clock().now().nanoseconds / 1e9

        linear_vel = self.move_pid.compute(
            self.target_distance, distance_traveled, current_time
        )

        twist = Twist()
        twist.linear.x = linear_vel
        self.cmd_vel_pub.publish(twist)

        if abs(self.target_distance - distance_traveled) < self.move_pid.deadzone:
            self.get_logger().info('Linear distance goal reached successfully!')
            self.stop_robot()
            self.is_linear_goal_reached = True

    def execute_yaw_callback(self, goal_handle):
        self.get_logger().info('New rotation goal received. Suspending linear movement...')
        self.is_rotating = True

        target_rotation = goal_handle.request.yaw_target
        final_angle = self.yaw_pid.normalize(target_rotation + self.current_yaw)

        self.yaw_pid.reset()
        self.yaw_pid.take_target(final_angle)

        feedback = YawRobot.Feedback()
        result = YawRobot.Result()
        twist_vel = Twist()

        rate = self.create_rate(10)
        loop_counter = 0
        success = False

        while rclpy.ok():
            loop_counter += 1
            now_sec = self.get_clock().now().nanoseconds / 1e9

            if (
                self.last_odom_time is None
                or (now_sec - self.last_odom_time) > 0.5
            ):
                self.get_logger().warn('No /odom feedback — aborting yaw action.')
                success = False
                break

            if loop_counter > 100:
                self.get_logger().warn('Rotation action timed out.')
                success = False
                break

            error_now = self.yaw_pid.normalize(final_angle - self.current_yaw)
            output = self.yaw_pid.compute(
                final_angle, self.current_yaw, current_time=now_sec
            )

            if abs(error_now) < self.yaw_pid.deadzone:
                success = True
                break

            twist_vel.angular.z = output
            self.cmd_vel_pub.publish(twist_vel)

            feedback.current_yaw = self.current_yaw
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        self.stop_robot()
        self.is_rotating = False

        result.success = success
        if success:
            goal_handle.succeed()
            self.get_logger().info('Rotation successful.')
        else:
            goal_handle.abort()

        return result

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


# ------------------ Main Execution ------------------

def main(args=None):
    rclpy.init(args=args)
    node = RobotMoverNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()