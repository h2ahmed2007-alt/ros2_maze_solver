#for the odometry logic part
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from packageName.action import YawRobot
from rclpy.action import ActionServer


# to keep angle within -90 to 90 in rads
def normalize_angle(angle):
  return math.atan2(math.sin(angle), math.cos(angle))

#using /odom to find the angle of orientation
#calculations to turn the x y z w to angle
def angleoforientation(q):
     sinYaw = 2.0 * (q.w * q.z + q.x * q.y)
       cosYaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sinYaw, cosYaw)


# server code
class yawRobotActionServer(Node):
   def __init__(self):
    super().__init__('yaw_robot_server')
      self.current_yaw = 0.0 #to store the rotation angle
       #subscribe to odom topic
       self.odom_sub = self.create_subscription(
         Odometry, '/odom', self.odom_callback, 10
)

#publish on cmd_vel to send angular velocity for rotation
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action_server = ActionServer(
           self, YawRobot, 'yawRobot', execute_callback=self.execute_callback )

 #callback functions
    def odom_callback(self, msg):
      q = msg.pose.pose.orientation
     self.current_yaw = angleoforientation(q)

   def execute_callback(self, goal): 
       self.get_logger().info('New rotation goal.')

 #if no /odom topic:
         topicName = '/odom'

          topic_info = self.get_topic_names_and_types()
           topic_names = [name for name, types in topic_info]
       if topicName not in topic_names:
            goal.abort()
            result = YawRobot.Result()
             result.success = False
           return result

        #otherwise get the angle 
        Targetrotation = goal.request.yaw_target
          # angle is accumulatively calculated so add it ot current yaw
           final_angle = normalize_angle(Targetrotation + self.current_yaw)

         feedback = YawRobot.Feedback()
         result = YawRobot.Result()
        twist_vel = Twist()

        #angular velocity has direction
        if Targetrotation >= 0:
             twist_vel.angular.z = 1.0
        else:
            twist_vel.angular.z = -1.0 #to turn clockwise

         #timeout part :
        rate = self.create_rate(10)#sets frequency per second
        loop_counter = 0

         while rclpy.ok():
             loop_counter += 1

             if loop_counter > 100:#if it passed 10 seconds
                 self.get_logger().warn('action timed out ')
                result.success = False
                 break 


            self.cmd_vel_pub.publish(twist_vel)
            feedback.current_yaw = self.current_yaw
            goal.publish_feedback(feedback)

            rate.sleep()

        # stop the rotation
        twist_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_vel)

        if result.success:
            goal.succeed()
            self.get_logger().info('rotation successful')
        else:
          goal.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = yawRobotActionServer()
    rclpy.spin(node)


if __name__ == '__main__':
  main()