import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import random
import math

class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'pose_array', 10)
        self.min_distance = 30.0  # 设置最小距离阈值
        self.max_attempts = 100  # 最大尝试次数
        self.k_value = None  # 用于存储用户输入的 K 值

    def check_k_value_and_publish(self):
        if self.k_value == 'K':
            self.publish_pose_array()

    def publish_pose_array(self):
        pose_array = PoseArray()
        
        for _ in range(3):  # 每次发布 3 个 Pose
            attempt = 0
            x = 0
            y = 0
            while attempt < self.max_attempts:
                pose = Pose()
                pose.position.x = random.uniform(142, 166)
                x = pose.position.x
                pose.position.y = random.uniform(190, 320)
                y = pose.position.y
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0  # 四元数的默认值

                # 检查新生成的 pose 是否与已有 pose 过近
                if self.is_far_enough(pose, pose_array.poses):
                    pose_array.poses.append(pose)
                    print("其中一个坐标为：(", x, y, ")")
                    break

                attempt += 1

            if attempt == self.max_attempts:
                self.get_logger().warn('Failed to generate a pose that meets the distance requirement')

        self.publisher_.publish(pose_array)
        self.get_logger().info('已发布话题!!!!!!')
        print(" ")

    def is_far_enough(self, new_pose, existing_poses):
        for pose in existing_poses:
            distance = math.sqrt(
                (new_pose.position.x - pose.position.x) ** 2 +
                (new_pose.position.y - pose.position.y) ** 2
            )
            # print(distance)
            if distance < self.min_distance:
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()

    try:
        while rclpy.ok():
            k_value = input("输入 K 值随机生成基于相机坐标系的坐标: ").strip()
            node.k_value = k_value
            node.check_k_value_and_publish()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
