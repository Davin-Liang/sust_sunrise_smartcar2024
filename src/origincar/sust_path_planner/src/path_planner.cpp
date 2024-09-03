#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
// #include "ai_msgs/msg/perception_targets.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <utility>
#include <algorithm>

std::vector<std::vector<float>> mapMatrix_(200, std::vector<float>(500, 0.0f));

std::pair<int, int> inverse_perspective_point(int x, int y) 
{
    std::vector<std::vector<double>> matrix = 
    {
        {-8.46875535e-02, -9.04096702e-03, 2.95338002e+01},
        {-2.94831011e-17, 5.97390039e-02, -3.66614618e+01},
        {-0.00000000e+00, -7.72487947e-03, 1.00000000e+00}
    };

    std::vector<double> homogeneous_point = {static_cast<double>(x), static_cast<double>(y), 1.0};
    std::vector<double> transformed_point(3, 0.0);

    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            transformed_point[i] += matrix[i][j] * homogeneous_point[j];
        }
    }

    transformed_point[0] /= transformed_point[2];
    transformed_point[1] /= transformed_point[2];

    int out_x = static_cast<int>(round(transformed_point[0]));
    int out_y = static_cast<int>(round(transformed_point[1]));

    return {out_x, out_y};
}

std::pair<double, double> local_to_global(double car_x, double car_y, double theta, double c_x, double c_y, int type) 
{
    c_y += 23.0;
    double cone_dis = (type == 0) ? sqrt(c_x * c_x + c_y * c_y) + 14.0 : sqrt(c_x * c_x + c_y * c_y);
    double global_x = car_x * 100.0 + cone_dis * cos(theta - atan2(c_x, c_y));
    double global_y = car_y * 100.0 + cone_dis * sin(theta - atan2(c_x, c_y));

    return {-global_x, global_y};
}

struct MyNode 
{
    std::pair<int, int> pos;
    double f;

    MyNode(std::pair<int, int> pos, double f) : pos(pos), f(f) {}

    bool operator<(const MyNode &other) const 
    {
        return f > other.f;
    }
};

double get_dis(const std::pair<int, int> &a, const std::pair<int, int> &b) 
{
    return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
}

std::pair<int, int> find_nearest_free_point(const std::vector<std::vector<float>> &Map, std::pair<int, int> start) 
{
    int rows = Map.size();
    int cols = Map[0].size();

    for (int i = start.first; i < rows; ++i) 
    {
        for (int j = start.second; j < cols; ++j) 
        {
            if (Map[i][j] == 0.0f) 
            {
                return {i, j};
            }
        }
    }
    return start;
}

void set_circle(std::vector<std::vector<float>>& mapMatrix_, int x, int y, int radius) 
{
    int rows = mapMatrix_.size();
    int cols = mapMatrix_[0].size();

    // 遍历矩阵的所有元素
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            // 计算当前元素 (i, j) 到中心点 (x, y) 的欧氏距离
            float distance = std::sqrt(std::pow(i - x, 2) + std::pow(j - y, 2));

            // 如果距离小于等于半径，则设置为 1.0
            if (distance <= radius) 
            {
                mapMatrix_[i][j] = 1.0f;
            }
        }
    }
}


std::vector<std::pair<int, int>> PathPlanner(
    const std::vector<std::vector<float>> &Map,
    const std::pair<int, int> &start,
    const std::pair<int, int> &des,
    int sign,
    double yaw
) 
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<MyNode> priority_queue;
    std::pair<int, int> now_loc;
    std::pair<int, int> now_des;
    std::pair<int, int> now_start;

    const int map_height = Map.size();
    const int map_width = Map[0].size();

    if (sign == 0 || sign == 1) 
    {
        if (start.second > des.second) 
        {
            return {des};
        }
    }

    now_start = {std::min(start.first, map_height - (sign==0||sign==1?35:31)), std::min(start.second, map_width - 50)};
    now_start = {std::max(now_start.first, 0), std::max(now_start.second, (sign==0||sign==1?6:0))};
    now_des = {std::min(des.first, map_height - (sign==0||sign==1?35:31)), std::min(des.second, map_width - 50)};
    now_des = {std::max(now_des.first, 20), std::max(now_des.second, 0)};

    if (Map[now_start.first][now_start.second] != 0.0f) 
    {
        now_start = find_nearest_free_point(Map, now_start);
    }

    now_loc = now_des;
    priority_queue.push(MyNode(now_loc, 0.0));

    std::vector<std::vector<std::pair<int, int>>> pre(map_height, std::vector<std::pair<int, int>>(map_width, {-1, -1}));
    std::vector<std::vector<double>> visited(map_height, std::vector<double>(map_width, -1.0));
    visited[now_loc.first][now_loc.second] = 0.0;
    std::pair<int, int> tmp = now_start;

    int cnt = 0;
    while (!priority_queue.empty()) 
    {
        cnt++;
        if (cnt > 40000) 
        {
            tmp = now_start;
            break;
        }

        MyNode current_node = priority_queue.top();
        priority_queue.pop();
        now_loc = current_node.pos;

        if (get_dis(now_loc, now_start) <= 20.0) 
        {
            tmp = now_loc;
            break;
        }

        for (int angle = 0; angle < 360; angle += 30) 
        {
            int i = static_cast<int>(now_loc.first + 7 * cos(angle * M_PI / 180.0));
            int j = static_cast<int>(now_loc.second + 11 * sin(angle * M_PI / 180.0));

            if (i >= (sign==0||sign==1?0:24) && i < map_height-35 && j >= (sign==0||sign==1?15:0) && j < map_width-50) 
            {
                if ( Map[i][j] == 0.0f && (visited[i][j] == -1.0 || visited[now_loc.first][now_loc.second] + 10.0 < visited[i][j]) ) 
                    {
                        double now_dis=get_dis({i, j}, now_start);
                        if(sign==0 && now_dis<=35 && fabs(atan2(i-now_start.first,j-now_start.second)-yaw)>40 * M_PI / 180.0)
                            continue;
                        if((sign==5||sign==6)&&i<=now_des.first+4)
                            continue;
                        pre[i][j] = now_loc;
                        double f = visited[now_loc.first][now_loc.second] + 10.0 + now_dis;
                        priority_queue.push(MyNode({i, j}, f));
                        visited[i][j] = visited[now_loc.first][now_loc.second] + 10.0;
                }
            }
        }
    }
    
    if (pre[tmp.first][tmp.second].first == -1) 
    {
        path.push_back(tmp);
    }
    while (pre[tmp.first][tmp.second].first != -1 && get_dis({tmp.first,tmp.second},now_des)>15.0) 
    {
        if (sign == 0 || sign == 1) 
        {
            path.push_back(tmp);
        } 
        else 
        {
            path.push_back({tmp.first, tmp.second});
        }
        tmp = pre[tmp.first][tmp.second];
    }

    path.push_back(tmp);
    std::reverse(path.begin(), path.end());

    return path;
}

class PathPlannerNode : public rclcpp::Node 
{
public:
    PathPlannerNode() : Node("path_planner_node") 
    {
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/goal_pose", 10, std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

        car_position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathPlannerNode::odom_callback, this, std::placeholders::_1));

        obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "pose_array", 10, std::bind(&PathPlannerNode::pose_array_callback, this, std::placeholders::_1));
            
        obstacle_list_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_list", 10);
    

        // 发布路径的主题
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/planned_path", 10);
        RCLCPP_INFO(this->get_logger(), "已启动路径规划节点!!!!!!");
    }

private:
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
    {
        std::cout << "================================================================" << std::endl;
        RCLCPP_INFO(this->get_logger(), "目前视野中有 %ld 个障碍物!!!!!!", msg->poses.size());
        geometry_msgs::msg::PoseArray obstacle_list;

        for (size_t i = 0; i < msg->poses.size(); ++i) 
        {
            const auto &pose = msg->poses[i];
            RCLCPP_INFO(this->get_logger(), "设置障碍物 %ld 的位置......", i);

            std::pair<int, int> obstacle_relative_point = inverse_perspective_point(static_cast<int>(pose.position.y), 
                                                                                    static_cast<int>(pose.position.x));
            RCLCPP_INFO(this->get_logger(), "相机坐标系中的障碍物坐标为: (%.2f, %.2f)", pose.position.x, pose.position.y);
            RCLCPP_INFO(this->get_logger(), "car 坐标系中的障碍物坐标为: (%d, %d)", obstacle_relative_point.first, obstacle_relative_point.second);
            // std::cout << "相机坐标系中的障碍物坐标为: (" << pose.position.x << ", " << pose.position.y << ")" <<std::endl;
            // std::cout << "obstacle_relative_point = " << obstacle_relative_point.first << " " << obstacle_relative_point.second <<std::endl;
            std::pair<double, double> obstacle_absolute_point = local_to_global(0.0, 
                                                                                0.0,
                                                                                1.57,
                                                                                static_cast<double>(obstacle_relative_point.first),
                                                                                static_cast<double>(obstacle_relative_point.second),
                                                                                0);
            RCLCPP_INFO(this->get_logger(), "odom 坐标系中的障碍物坐标为: (%.2f, %.2f)", obstacle_absolute_point.first, obstacle_absolute_point.second);                                  
            // std::cout << "obstacle_absolute_point = " << obstacle_absolute_point.first << " " << obstacle_absolute_point.second <<std::endl;

            geometry_msgs::msg::Pose obstacle_pose;
            obstacle_pose.position.x = obstacle_absolute_point.first;
            obstacle_pose.position.y = obstacle_absolute_point.second;
            obstacle_pose.position.z = 0.0;  // 如果有高度信息可以修改
            obstacle_pose.orientation.w = 1.0;  // 设置为单位四元数，表示没有旋转

            // 添加到 PoseArray 中
            obstacle_list.poses.push_back(obstacle_pose);

            set_circle(mapMatrix_, 
                        static_cast<int>(obstacle_absolute_point.first), 
                        static_cast<int>(obstacle_absolute_point.second),
                        40);
        }
        RCLCPP_INFO(this->get_logger(), "障碍物的绝对坐标已发布出去!!!!!!");
        RCLCPP_INFO(this->get_logger(), "等待接收目标点进行路径规划......");
        obstacle_list_publisher_->publish(obstacle_list);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received odom data: Position [x: %f, y: %f, z: %f]",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
    }

    void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) 
    {
        // 接收终点位置
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        goal_ = {static_cast<int>(msg->position.x), static_cast<int>(msg->position.y)};
        plan_path();
        std::cout << "===============================================================" << std::endl;
    }

    void plan_path() 
    {
        if (mapMatrix_.empty()) 
            return;

        int sign = 0; 
        double yaw = 0.0;

        auto path = PathPlanner(mapMatrix_, start_, goal_, sign, yaw);

        // 将路径发布出去
        geometry_msgs::msg::PoseArray path_msg;
        for (const auto& p : path) 
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.first;
            pose.position.y = p.second;
            path_msg.poses.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "路径坐标已发布出去!!!!!!");
        path_pub_->publish(path_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_list_publisher_;

    std::pair<int, int> start_;
    std::pair<int, int> goal_;
};



int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
