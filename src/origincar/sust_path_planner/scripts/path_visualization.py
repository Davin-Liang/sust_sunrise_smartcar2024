import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt

class PoseArraySubscriber(Node):
    def __init__(self):
        super().__init__('pose_array_subscriber')
        # 订阅 pose_array 话题
        self.subscription_pose_array = self.create_subscription(
            PoseArray,
            'planned_path',
            self.pose_array_callback,
            10)
        
        # 订阅 obstacle_list 话题
        self.subscription_obstacle_list = self.create_subscription(
            PoseArray,
            'obstacle_list',
            self.obstacle_list_callback,
            10)

        self.pose_x_data = []
        self.pose_y_data = []
        self.obstacle_x_data = []
        self.obstacle_y_data = []

        self.get_logger().info("已启动路径绘制节点......")

    def pose_array_callback(self, msg):
        # 清空上次的数据
        self.get_logger().info("已接收到路径坐标信息，进行画图!!!!!!")
        print("================================================================")
        self.pose_x_data.clear()
        self.pose_y_data.clear()

        # 遍历 PoseArray 中的每一个 Pose，提取 x 和 y 坐标
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            self.pose_x_data.append(x)
            self.pose_y_data.append(y)

        # 更新图形
        self.plot_data()

    def obstacle_list_callback(self, msg):
        # 清空上次的数据
        print("================================================================")
        self.get_logger().info("已接收到障碍物的绝对坐标信息!!!!!!")
        self.get_logger().info("等待接收路径坐标信息......")
        self.obstacle_x_data.clear()
        self.obstacle_y_data.clear()

        # 遍历 PoseArray 中的每一个 Pose，提取 x 和 y 坐标
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            self.obstacle_x_data.append(x)
            self.obstacle_y_data.append(y)

        # 更新图形
        # self.plot_data()

    def plot_data(self):
        plt.figure()
        # 绘制轨迹折线图
        plt.plot(self.pose_y_data, self.pose_x_data, marker='o', linestyle='-', color='b', label='Path Points')
        
        # 绘制障碍物点
        plt.scatter(self.obstacle_y_data, self.obstacle_x_data, color='r', label='Obstacles')

        plt.scatter(self.obstacle_y_data, self.obstacle_x_data, color='r', s=30**2, edgecolor='r', facecolors='none')
        
        plt.title('Path Points and Obstacles')
        plt.xlabel('y Coordinate')
        plt.ylabel('x Coordinate')
        plt.grid(True)
        plt.axis('equal')  # 保持 X 和 Y 轴的刻度标准相同
        plt.legend()  # 添加图例
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    pose_array_subscriber = PoseArraySubscriber()
    
    try:
        rclpy.spin(pose_array_subscriber)
    except KeyboardInterrupt:
        pass

    pose_array_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
