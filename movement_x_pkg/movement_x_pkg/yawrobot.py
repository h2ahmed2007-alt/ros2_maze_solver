import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from robot_actions.action import YawRobot
from movement_x_pkg.pid_control import pidcontrol


class YawRobotActionServer(Node):
  def __init__(self):
    super().__init__('yaw_robot_server')
    self.current_yaw = 0.0
    self.last_odom_time = None  # watchdog
    self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.pid = pidcontrol(
            kp=1.0, ki=0.0, kd=0.1,
            min_out=-1.5, max_out=1.5,
            i_max=1.0,
            deadzone=0.02,   
            is_angle=True
        )
    self.yaw_pid_sub = self.create_subscription(Float32MultiArray, '/yaw_pid', self.yaw_pid_callback, 10)
    self._action_server = ActionServer(self, YawRobot, 'yawRobot', execute_callback=self.execute_callback)

  def yaw_pid_callback(self, msg):
    if len(msg.data) < 3:
      self.get_logger().warn('Received malformed /yaw_pid message, ignoring.')
      return
      self.pid.kp = msg.data[0]
      self.pid.ki = msg.data[1]
      self.pid.kd = msg.data[2]
      self.get_logger().info(f'Updated yaw PID gains: kp={self.pid.kp}, ki={self.pid.ki}, kd={self.pid.kd}')

  def odom_callback(self, msg):
    q = msg.pose.pose.orientation
    self.current_yaw = self.angle_of_orientation(q)
    self.last_odom_time = time.time()

  @staticmethod
  def angle_of_orientation(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)

  def execute_callback(self, goal_handle):
    self.get_logger().info('New rotation goal.')
    target_rotation = goal_handle.request.yaw_target
    final_angle = self.pid.normalize(target_rotation + self.current_yaw)
    self.pid.reset()
    self.pid.take_target(final_angle)

    feedback = YawRobot.Feedback()
    result = YawRobot.Result()
    twist_vel = Twist()

    rate = self.create_rate(10)
    loop_counter = 0
    success = False

    while rclpy.ok():
      loop_counter += 1     
      if self.last_odom_time is None or (time.time() - self.last_odom_time) > 0.5:
        self.get_logger().warn('No /odom feedback — aborting for safety.')
        success = False
        break

      if loop_counter > 100:
        self.get_logger().warn('Rotation action timed out.')
        success = False
        break

        error_now = self.pid.normalize(final_angle - self.current_yaw)
        output = self.pid.compute(final_angle, self.current_yaw, current_time=time.time())

      if output == 0.0 and abs(error_now) < self.pid.deadzone:
        success = True
        break

        twist_vel.angular.z = output
        self.cmd_vel_pub.publish(twist_vel)

        feedback.current_yaw = self.current_yaw
        goal_handle.publish_feedback(feedback)

        rate.sleep()

    twist_vel.angular.z = 0.0
    self.cmd_vel_pub.publish(twist_vel)

    result.success = success
    if success:
      goal_handle.succeed()
      self.get_logger().info('Rotation successful.')
    else:
      goal_handle.abort()

    return result


def main(args=None):
    rclpy.init(args=args)
    node = YawRobotActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
