#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        
        # 发布器：循环发布火力决策
        self.publisher_ = self.create_publisher(String, 'fire_command', 10)
        
        # 订阅 cmdtreeautoaim 话题（来自 setcmdtree）
        self.subscriber_ = self.create_subscription(
            String,
            'cmdtreeautoaim',
            self.message_callback,
            10
        )
        
        # 每0.2秒循环发布
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.current_message = ""
        
        self.get_logger().info('Number Publisher has been started')
        self.get_logger().info('订阅话题: /cmdtreeautoaim')
        
    def message_callback(self, msg):
        self.current_message = msg.data
        self.get_logger().info(f'Cockroach 收到火力决策: {msg.data}')
    
    def timer_callback(self):
        if self.current_message:
            msg = String()
            msg.data = self.current_message
            self.publisher_.publish(msg)


def main(args=None):
    ascii_art = r"""......\  /.......\  /.......\  /.......\  /.......\  /........
......_  _......._  _......._  _......._  _......._  _........
.....(.o o.)....(.o o.)....(.o o.)....(.o o.)....(.o o.)......
..../..V.V..\../..V.V..\../..V.V..\../..V.V..\../..V.V..\.....
.....\_/ \_/....\_/ \_/....\_/ \_/....\_/ \_/....\_/ \_/......
Cockroach invasion!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"""

    print(ascii_art)

    rclpy.init(args=args)
    node = NumberPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
