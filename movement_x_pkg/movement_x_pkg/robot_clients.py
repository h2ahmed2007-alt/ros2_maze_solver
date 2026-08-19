import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_actions.action import Movementros2
from robot_actions.action import YawRobot
import math


class client(Node):
    def __init__(self):
        super().__init__('move_robot_client')
        self.move_client = ActionClient(
            self,
            Movementros2,     # the move_x action type
            'move_robot_x'
        )
        self.yaw_client = ActionClient(
            self,
            YawRobot,        # the yaw action in the interface file
            'yawRobot'       # end point likewise the one in the server
        )

    # ---------------- MOVE ----------------
    def send_goal(self, distance_value):
        self.get_logger().info('waiting for server')
        self.move_client.wait_for_server(timeout_sec=5.0)

        goal_msg = Movementros2.Goal()
        goal_msg.distance = float(distance_value)

        self.send_goal_future = self.move_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("goal is accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        else:
            self.get_logger().info("goal is rejected")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(str(feedback.current_distance))

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("success")
        else:
            self.get_logger().error("failure")

    # ---------------- YAW ----------------
    def send_goal_yaw(self, rotation_value):
        # made general so solve_maze decides whether it's 0.0 or 90.0 deg
        rotation_value = math.radians(rotation_value)
        self.get_logger().info('waiting for server')
        self.yaw_client.wait_for_server(timeout_sec=5.0)

        goal_msg = YawRobot.Goal()
        goal_msg.yawTarget = rotation_value

        self.send_goal_yaw_future = self.yaw_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback_yaw
        )
        self.send_goal_yaw_future.add_done_callback(self.goal_response_callback_yaw)

    def goal_response_callback_yaw(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("goal is accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback_yaw)
        else:
            self.get_logger().info("goal is rejected")

    def feedback_callback_yaw(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(str(feedback.currentYaw))

    def result_callback_yaw(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("success")
        else:
            self.get_logger().error("failure")


def main():
    rclpy.init()
    node = client()
    node.send_goal(1.0)       # test: move 1.0 meter forward
    node.send_goal_yaw(90.0)  # test: rotate 90 degrees
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
