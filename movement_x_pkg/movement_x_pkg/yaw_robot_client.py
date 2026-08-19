import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_actions.action import YawRobot      ## robot_interfaces has to be made in the pkg
import math    

class yaw_client(Node): #the yaw node class and we inhereted from Node

    def __init__(self):
        super().__init__('yaw_robot_client')   
        self.yaw_client = ActionClient(
            self,
            YawRobot,        # the yaw action in the interface file
             'yawRobot'     #end point likewise the one in the server
        )

    def send_goal(self, rotation_value):   # to send the call to the server, made it general so the solve_maze decides whether its 0.0 or 90.0 deg
        

        rotation_value = math.radians(rotation_value)
        self.get_logger().info('waiting for server')  

        self.yaw_client.wait_for_server(timeout_sec=5.0)   # waiting for the server to work max time 5 sec

        goal_msg = YawRobot.Goal() #making the goal object

        goal_msg.yawTarget = rotation_value  

        self.send_goal_future = self.yaw_client.send_goal_async(goal_msg, feedback_callback= self.feedback_callback) #sending the goal to the server, using the feedback func we made to recieve the feedback

        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):  # a func to know the future response 

        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info("goal is accepted")

            result_future = goal_handle.get_result_async() #if the goal is accepted, we will request the result

            result_future.add_done_callback(self.result_callback) #when the result is ready result func will work

        else :
            self.get_logger().info("goal is rejected")


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(str(feedback.currentYaw)) #to get the feed back,turned it into str so get_logger can work

    def result_callback(self, future):    #to know wether the goal was success or not
        result = future.result().result

        if result.success :
            self.get_logger().info("success")

        else:
            self.get_logger().error("failure")

def main():

    rclpy.init()
    node = yaw_client()
    node.send_goal(0.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

