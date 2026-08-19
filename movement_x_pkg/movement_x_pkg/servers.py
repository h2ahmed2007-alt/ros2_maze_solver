import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_actions.action import Movementros2, YawRobot


# to keep angle within -90 to 90 in rads
def normalize_angle(angle):
  return math.atan2(math.sin(angle), math.cos(angle))


# using /odom to find the angle of orientation
# calculations to turn the x y z w to angle
def angleoforientation(q):
  sinYaw = 2.0 * (q.w * q.z + q.x * q.y)
  cosYaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(sinYaw, cosYaw)


class MergedServerNode(Node):

  def __init__(self):
    super().__init__('merged_server_node')
    self.reentrant_group = ReentrantCallbackGroup()

    self.yaw_action_server = ActionServer(
        self,
        YawRobot,
        'yawRobot',
        execute_callback=self.execute_callback_yaw,
        callback_group=self.reentrant_group,
    )

    self.current_x = 0.485
    self.current_y= 0.5   
    self.current_yaw = 0.0  # to store the rotation angle
    self.move_action_server = ActionServer(
        self,
        Movementros2,
        'move_robot_x',
        self.execute_callback_move,
        callback_group=self.reentrant_group,
    )

    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    self.odom_sub = self.create_subscription(
        Odometry,
        '/odom',
        self.odom_callback,
        10,
        callback_group=self.reentrant_group,
    )

  def odom_callback(self, msg):
    q = msg.pose.pose.orientation
    self.current_yaw = angleoforientation(q)
    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y

  def execute_callback_move(self, goal_handle):
    target_distance = goal_handle.request.distance
    start_x = self.current_x
    start_y = self.current_y

    twist = Twist()
    twist.linear.x = 0.4
    rate = self.create_rate(10)

    while rclpy.ok():
      distance_traveled = math.sqrt ((self.current_x - start_x) **2 + (self.current_y - start_y) **2)
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
    # stop the rotation
    twist.angular.z = 0.0
    self.cmd_vel_pub.publish(twist)

  def execute_callback_yaw(self, goal_handle):
    self.get_logger().info('New rotation goal.')

    # if no /odom topic:
    topicName = '/odom'

    topic_info = self.get_topic_names_and_types()
    topic_names = [name for name, types in topic_info]
    if topicName not in topic_names:
      goal_handle.abort()
      result = YawRobot.Result()
      result.success = False
      return result

    # otherwise get the angle
    Targetrotation = goal_handle.request.yaw_target
    # angle is accumulatively calculated so add it ot current yaw
    final_angle = normalize_angle(Targetrotation + self.current_yaw)

    feedback = YawRobot.Feedback()
    result = YawRobot.Result()
    twist_vel = Twist()

    # angular velocity has direction
    if Targetrotation >= 0:
      twist_vel.angular.z = 1.0
    else:
      twist_vel.angular.z = -1.0  # to turn clockwise

    # timeout part :
    start_time = time.time()
    timeout_limit = 10.0  # seconds

    rate = self.create_rate(10)  # Check 10 times per second

    while rclpy.ok():
      if (time.time() - start_time) > timeout_limit:
        self.get_logger().warn('action timed out.')
        result.success = False
        break

      error = normalize_angle(final_angle - self.current_yaw)

      if abs(error) < 0.05:
        result.success = True
        break

      self.cmd_vel_pub.publish(twist_vel)
      feedback.current_yaw = self.current_yaw
      goal_handle.publish_feedback(feedback)

      rate.sleep()

    self.stop_robot()

    if result.success:
      goal_handle.succeed()
      self.get_logger().info('rotation successful')
    else:
      goal_handle.abort()

    return result


def main(args=None):
  rclpy.init(args=args)
  node = MergedServerNode()
  executor = MultiThreadedExecutor()
  executor.add_node(node)
  try:
    executor.spin()
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()