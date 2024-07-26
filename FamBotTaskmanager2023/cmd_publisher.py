#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_msgs.msg import CVinstructions

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.publisher_ = self.create_publisher(CVinstructions, 'vision_command_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        cmd_input = input("请输入cmd: ")
        msg = CVinstructions()
        msg.cmd = int(cmd_input)
        self.publisher_.publish(msg)
        self.get_logger().info(f"发布了cmd: {msg.cmd}")

def main(args=None):
    rclpy.init(args=args)
    cmd_publisher = CmdPublisher()
    rclpy.spin(cmd_publisher)
    cmd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
