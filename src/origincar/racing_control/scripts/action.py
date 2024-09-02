import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BoolPublisherNode(Node):

    def __init__(self):
        super().__init__('bool_publisher_node')
        self.publisher_ = self.create_publisher(Bool, '/start_car', 10)

    def publish_message(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = BoolPublisherNode()

    user_input = input("请输入一个整数或者 'true'：")
    if user_input.lower() == 'true':
        node.publish_message(True)
        rclpy.shutdown()  # 发布后关闭节点
    else:
        print("输入的不是 'true'，程序将退出。")

if __name__ == '__main__':
    main()