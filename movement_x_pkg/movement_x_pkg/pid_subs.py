import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class pid_subscribers(Node) :
    def __init__(self):
        super().__init__("pid_subscriber")

        self.kp_yaw = 0.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

        self.kp_move = 0.0
        self.ki_move = 0.0
        self.kd_move = 0.0

        self.yaw_pid_subscriber = self.create_subscription(
        Float32MultiArray,
        "/yaw_pid",
        self.yaw_pid_callback,
        10
        )

        self.move_pid_subscriber = self.create_subscription(
        Float32MultiArray,
        "/move_pid",
        self.move_pid_callback,
        10
        )
    def yaw_pid_callback(self,msg):

        self.kp_yaw = msg.data[0]
        self.ki_yaw = msg.data[1]
        self.kd_yaw = msg.data[2]

    def move_pid_callback(self,msg):

        self.kp_move = msg.data[0]
        self.ki_move = msg.data[1]
        self.kd_move = msg.data[2]        


def main(args=None):
    rclpy.init(args=args)

    node = pid_subscribers()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()