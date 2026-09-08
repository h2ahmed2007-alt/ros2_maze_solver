import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class pid_publishers(Node) :
    def __init__(self):
        super().__init__("pid_publisher")

        self.yaw_pid_publiser = self.create_publisher(
        Float32MultiArray,
        "/yaw_pid",
        10
        )

        self.move_pid_publiser = self.create_publisher(
        Float32MultiArray,
        "/move_pid",
        10
        )
    


def main(args=None):
    rclpy.init(args=args)

    node = pid_publishers()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()