// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "racing_control/racing_control.h"

#include <unistd.h>
#include <chrono> // 包含 std::chrono::seconds
#include <iostream>
#include <stdlib.h>


RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options)
{
  // pid_(0.004, 0.0, 0.01, 0.0, 0.05, 1.0)
  if (!msg_process_)
    msg_process_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::MessageProcess, this));

  // 避障线程
  if ( !avoid_thread_ )
    avoid_thread_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::avoid_obstacle, this));
  /* PID 参数 */
  this->declare_parameter<float>("P_param", pid_.kp_);
  this->get_parameter<float>("P_param", pid_.kp_);
  this->declare_parameter<float>("I_param", pid_.ki_);
  this->get_parameter<float>("I_param", pid_.ki_);
  this->declare_parameter<float>("D_param", pid_.kd_);
  this->get_parameter<float>("D_param", pid_.kd_);
  this->declare_parameter<float>("target_param", pid_.target_position_);
  this->get_parameter<float>("target_param", pid_.target_position_);
  this->declare_parameter<float>("integral_limit_param", pid_.integral_limit_);
  this->get_parameter<float>("integral_limit_param", pid_.integral_limit_);
  this->declare_parameter<float>("max_out_param", pid_.max_out_);
  this->get_parameter<float>("max_out_param", pid_.max_out_);
  /* 避障参数 */
  this->declare_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->declare_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);
  this->declare_parameter<float>("avoid_ratio_coefficient_1", avoid_ratio_coefficient_1_);
  this->get_parameter<float>("avoid_ratio_coefficient_1", avoid_ratio_coefficient_1_);
  this->declare_parameter<float>("avoid_ratio_coefficient_2", avoid_ratio_coefficient_2_);
  this->get_parameter<float>("avoid_ratio_coefficient_2", avoid_ratio_coefficient_2_);
  this->declare_parameter<int>("safe_distance", safe_distance_);
  this->get_parameter<int>("safe_distance", safe_distance_);
  this->declare_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
  this->get_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
  /* 寻线参数 */
  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  /* 避障时间 */
  this->declare_parameter<float>("avoid_time_1", avoid_time_1_);
  this->get_parameter<float>("avoid_time_1", avoid_time_1_);
  this->declare_parameter<float>("avoid_time_2", avoid_time_2_);
  this->get_parameter<float>("avoid_time_2", avoid_time_2_);
  this->declare_parameter<float>("avoid_time_3", avoid_time_3_);
  this->get_parameter<float>("avoid_time_3", avoid_time_3_);
  this->declare_parameter<float>("avoid_time_4", avoid_time_4_);
  this->get_parameter<float>("avoid_time_4", avoid_time_4_);
  this->declare_parameter<float>("avoid_time_5", avoid_time_5_);
  this->get_parameter<float>("avoid_time_5", avoid_time_5_);
  this->declare_parameter<float>("avoid_time_6", avoid_time_6_);
  this->get_parameter<float>("avoid_time_6", avoid_time_6_);
  /* 跟踪参数 */
  this->declare_parameter<float>("trace_linear_speed", trace_linear_speed_);
  this->get_parameter<float>("trace_linear_speed", trace_linear_speed_);
  this->declare_parameter<int>("qr_area_threshold", qr_area_threshold_);
  this->get_parameter<int>("qr_area_threshold", qr_area_threshold_);
  this->declare_parameter<float>("confidence_threshold", confidence_threshold_);
  this->get_parameter<float>("confidence_threshold", confidence_threshold_);


  point_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>("racing_track_center_detection", 10,
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1)); 

  target_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>("hobot_dnn_detection", 10,
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1)); 

  control_command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("sign4return", 10,
      std::bind(&RacingControlNode::control_command_callback_, this, std::placeholders::_1));

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);

  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

RacingControlNode::~RacingControlNode()
{
  if (msg_process_ && msg_process_->joinable()) 
  {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }

  if (avoid_thread_ && avoid_thread_->joinable()) 
  {
    avoid_thread_->join();
    avoid_thread_ = nullptr;
  }

  if (spinning_thread_ && spinning_thread_->joinable()) 
  {
    spinning_thread_->join();
    spinning_thread_ = nullptr;
  }

  {
  std::unique_lock<std::mutex> lock(point_target_mutex_);
  while (!point_queue_.empty())
    point_queue_.pop();
  }

  {
  std::unique_lock<std::mutex> lock(point_target_mutex_);
  while (!targets_queue_.empty())
    targets_queue_.pop();
  }
}

void RacingControlNode::start() 
{
  // 创建单线程执行器
  auto executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // 将节点添加到执行器中
  executor_->add_node(shared_from_this());

  // 启动执行器（异步方式）
  std::promise<void> promise;
  auto future = promise.get_future();
  std::thread([this, &promise, executor_]() {
      executor_->spin();
      promise.set_value();
  }).detach();

  // 等待执行器启动完成（非必须步骤）
  future.wait();
}

void RacingControlNode::control_command_callback_(const std_msgs::msg::Int32::SharedPtr msg)
{
  control_command = msg->data;
  if ( msg->data == 5 )
  {
    std::cout << "二维码识别成功，立即发布停车指令......" << std::endl;
    scan_qr_success = true;
  }
    

  return;
}

void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg){
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    point_queue_.push(point_msg);
    if (point_queue_.size() > 1) {
      point_queue_.pop();
    }
  }
  return;
}

void RacingControlNode::spin_node(void)
{
  while (1)
  {
    // 创建单独的 executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->shared_from_this());
    executor.spin();
  }

}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg)
{
  sub_target_ = true;

  for ( const auto &target : targets_msg->targets )
  {
    if ( target.rois[0].confidence > confidence_threshold_ )
    {
      if (target.type == "qr_code\r")
      {
        qr_vision.central_point[0] = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
        qr_vision.central_point[1] = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
        qr_vision.area = target.rois[0].rect.width * target.rois[0].rect.height;
      }
      else if (target.type == "parking_sign\r")
      {
        home_vision.central_point[0] = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
        home_vision.central_point[1] = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
        home_vision.area = target.rois[0].rect.width * target.rois[0].rect.height;

        control_command = 7;
        start_home = true;
        start_avoid = false;
      }

      float area = 0.0;
      std::string type = "left";

      if (target.type == "left\r")
      {
        float temp = target.rois[0].rect.width * target.rois[0].rect.height;
        if ( temp > area )
        {
          area = temp;
          type = "left";
        }
      }
      else if (target.type == "right\r")
      {
        float temp = target.rois[0].rect.width * target.rois[0].rect.height;
        if ( temp > area )
        {
          area = temp;
          type = "right";
        }
      }
      if (area > 0)
      {
        ori.area = area;
        ori.type = type;
        std::cout << "离车最近的障碍物在线的：" << ori.type << std::endl;
      }
    } 
  }

  std::unique_lock<std::mutex> lock(point_target_mutex_);
  targets_queue_.push(targets_msg);
  if (targets_queue_.size() > 1)
    targets_queue_.pop();

  return;
}

void RacingControlNode::avoid_obstacle(void)
{
  while (1)
  {
    if (start_avoid)
    {
      std::cout << "开始避障!!!!!!" << std::endl;
      draw_circle();
      std::cout << "结束避障!!!!!!" << std::endl;
      if (start_avoid != false)
        control_command = 6;
      else
      {
        std::cout << "避障流程已被打断!!!!!!" << std::endl;
      }
      start_avoid = false;
    }
  }

}

void RacingControlNode::MessageProcess(){
  while(process_stop_ == false)
  {
    this->get_parameter<float>("P_param", pid_.kp_);
    this->get_parameter<float>("I_param", pid_.ki_);
    this->get_parameter<float>("D_param", pid_.kd_);
    this->get_parameter<float>("target_param", pid_.target_position_);
    this->get_parameter<float>("integral_limit_param", pid_.integral_limit_);
    this->get_parameter<float>("max_out_param", pid_.max_out_);
    this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
    this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
    this->get_parameter<int>("bottom_threshold", bottom_threshold_);
    this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
    this->get_parameter<float>("confidence_threshold", confidence_threshold_);
    this->get_parameter<float>("avoid_time_1", avoid_time_1_);
    this->get_parameter<float>("avoid_time_2", avoid_time_2_);
    this->get_parameter<float>("avoid_time_3", avoid_time_3_);
    this->get_parameter<float>("avoid_time_4", avoid_time_4_);
    this->get_parameter<float>("avoid_time_5", avoid_time_5_);
    this->get_parameter<float>("avoid_time_6", avoid_time_6_);
    this->get_parameter<float>("trace_linear_speed", trace_linear_speed_);
    this->get_parameter<int>("qr_area_threshold", qr_area_threshold_);
    this->get_parameter<float>("avoid_ratio_coefficient_1", avoid_ratio_coefficient_1_);
    this->get_parameter<float>("avoid_ratio_coefficient_2", avoid_ratio_coefficient_2_);
    this->get_parameter<int>("safe_distance", safe_distance_);
    this->get_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
    this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);

    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty() && sub_target_ == false){
      auto point_msg = point_queue_.top();
      lock.unlock();

      if ( control_command == 6 )
        LineFollowing(point_msg->targets[0]);
      else if ( control_command == 5 )
        stop_car();

      lock.lock();
      point_queue_.pop();
    }
    if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true) 
    {
      auto point_msg = point_queue_.top();
      auto targets_msg = targets_queue_.top();
      point_queue_.pop();
      targets_queue_.pop();
      lock.unlock();

      if ( control_command == 6 )
      {
        if (targets_msg->targets.size() == 0)
          LineFollowing(point_msg->targets[0]);
        else 
        {
          for(const auto &target : targets_msg->targets)
          {
            if (target.type == "obstacle\r")
            {
              int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
              int distance = std::abs(320 - static_cast<int>(target.rois[0].rect.x_offset + target.rois[0].rect.width / 2));
              int area = target.rois[0].rect.width * target.rois[0].rect.height;
              if (bottom < bottom_threshold_ or area < obstacle_area_threshold_)
                LineFollowing(point_msg->targets[0]);
              else 
                if(target.rois[0].confidence > confidence_threshold_)
                {
                  if (distance < safe_distance_)
                  {
                    std::cout << "避障条件都满足!!!!!!" << std::endl;
                    main_target_ = target;
                    start_avoid = true; // 打开避障
                    control_command = 7; // 让小车脱离寻线控制
                  }
                  
                }
            }           
          }
        }
      }
      else if (control_command == 5)
      {
        std::cout << "停车中......" << std::endl;
        stop_car();
        start_trace = false;
        start_home = false;
        start_avoid = false;
        control_command = 7;
      }
        
      /* 跟踪控制 */
      if (start_trace)
      {
        if (scan_qr_success == true)
        {
          auto twist_msg = geometry_msgs::msg::Twist();
          publisher_->publish(twist_msg);
          start_trace = false;
          scan_qr_success = false;
        }
        if (start_trace)
          trace_central_point();
      }

      if (start_home)
      {
        trace_central_point();
      }


      if (start_detect)
      {
        if ( qr_vision.area > qr_area_threshold_ )
        {
          std::cout << "关闭寻线，开始跟踪二维码......" << std::endl;
          start_detect = false;
          start_avoid = false;
          start_trace = true;
          control_command = 7;
        }
      }

      if (control_command != 5)
      {
        if (start_avoid)
        {
          publisher_->publish(avoid_twist_);
          std::cout << "正在避障......" << std::endl;
        }
          
        if (start_trace or start_home)
        {
          publisher_->publish(trace_twist_);
          std::cout << "正在跟踪......" << std::endl;
        }
      }

      lock.lock();
    }
  }
}

void RacingControlNode::trace_central_point(void)
{
  int x = 0;
  int y = 0;
  if (start_trace)
  {
    x = int(qr_vision.central_point[0]);
    y = int(qr_vision.central_point[1]);
  }
  if (start_home)
  {
    x = int(home_vision.central_point[0]);
    y = int(home_vision.central_point[1]);
  }
  
  float temp = x - 320.0;
  // std::cout << temp << std::endl;
  float angular_z = pid_.compute(temp);
  trace_twist_.linear.x = trace_linear_speed_;
  trace_twist_.angular.z = angular_z;
}

void RacingControlNode::stop_car(void)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  publisher_->publish(twist_msg);
}

/* 避障画圆 */
void RacingControlNode::draw_circle(void)
{
  // int center_x = main_target_.rois[0].rect.x_offset + main_target_.rois[0].rect.width / 2;
  // float temp = center_x - 320.0;
  int temp = 1;
  if (ori.type == "left")
    temp = 1;
  else if (ori.type == "right")
    temp = -1;

  avoid_twist_.linear.x = avoid_linear_speed_;

  float angular_z = avoid_angular_ratio_; // 0.620455

  if (temp > 0)
    angular_z = angular_z;
  else
  angular_z = -angular_z;

  avoid_twist_.angular.z = angular_z;
  if (temp > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_1_));
  else
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_4_));

  avoid_twist_.angular.z = -angular_z * avoid_ratio_coefficient_1_;
  if (temp > 0)
  std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_2_));
  else
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_5_));

  avoid_twist_.angular.z = angular_z * avoid_ratio_coefficient_2_;
  if (temp > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_3_));
  else
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_6_));
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &point_msg)
{
  int x = int(point_msg.points[0].point[0].x);
  int y = int(point_msg.points[0].point[0].y);
  float temp = x - 320.0;

  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = pid_.compute(temp);

  twist_msg.linear.x = follow_linear_speed_;
  twist_msg.angular.z = angular_z;
    
  publisher_->publish(twist_msg);
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  // auto node = std::make_shared<RacingControlNode>("GetLineCoordinate");
  // node->start();

  while (1)
    ;
  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
