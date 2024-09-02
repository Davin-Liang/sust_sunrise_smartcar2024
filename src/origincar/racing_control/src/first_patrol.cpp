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

#include "racing_control/first_patrol.h"
#include <unistd.h>
#include <chrono> // 包含 std::chrono::seconds
#include <iostream>
#include <stdlib.h>
#include <ncurses.h>


RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options)
{
  // pid_(0.004, 0.0, 0.01, 0.0, 0.05, 1.0)
  if (!msg_process_)
    msg_process_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::MessageProcess, this));

  // 避障线程
  if ( !avoid_thread_ )
    avoid_thread_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::avoid_obstacle, this));

  if ( !key_thread_ )
    key_thread_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::checkKeyPress, this));

  auto pd_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
  auto pd_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
  auto pd_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();

  auto range_param = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
  range_param->from_value = 0.0;
  range_param->to_value = 0.5;
  range_param->step = 0.0001;
  pd_kp->floating_point_range.emplace_back(*range_param);

  range_param->from_value = 0.0;
  range_param->to_value = 0.1;
  range_param->step = 0.000001;
  pd_ki->floating_point_range.emplace_back(*range_param);

  range_param->from_value = 0.0;
  range_param->to_value = 0.1;
  range_param->step = 0.00001;
  pd_kd->floating_point_range.emplace_back(*range_param);

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

  this->declare_parameter<float>("P_angle_param", pid_angle_.kp_);
  this->get_parameter<float>("P_angle_param", pid_angle_.kp_);
  this->declare_parameter<float>("D_angle_param", pid_angle_.kd_);
  this->get_parameter<float>("D_angle_param", pid_angle_.kd_);

  this->declare_parameter<float>("max_out_angle_param", pid_angle_.max_out_);
  this->get_parameter<float>("max_out_angle_param", pid_angle_.max_out_);

  // this->get_parameter<float>("P_angle_param", pid_angle_.kp_);
  // this->get_parameter<float>("D_angle_param", pid_angle_.kd_);
  // this->get_parameter<float>("max_out_angle_param", pid_angle_.max_out_);

  this->declare_parameter<float>("P_angle_x_param", pid_angle_x_.kp_);
  this->get_parameter<float>("P_angle_x_param", pid_angle_x_.kp_);
  this->declare_parameter<float>("D_angle_x_param", pid_angle_x_.kd_);
  this->get_parameter<float>("D_angle_x_param", pid_angle_x_.kd_);

  this->declare_parameter<float>("max_out_angle_x_param", pid_angle_x_.max_out_);
  this->get_parameter<float>("max_out_angle_x_param", pid_angle_x_.max_out_);

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
  this->declare_parameter<float>("avoid_ratio_coefficient_3", avoid_ratio_coefficient_3_);
  this->get_parameter<float>("avoid_ratio_coefficient_3", avoid_ratio_coefficient_3_);
  this->declare_parameter<int>("safe_distance", safe_distance_);
  this->get_parameter<int>("safe_distance", safe_distance_);
  this->declare_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
  this->get_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
  /* 寻线参数 */
  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->declare_parameter<float>("speed_scaling_factor", speed_scaling_factor_);
  this->get_parameter<float>("speed_scaling_factor", speed_scaling_factor_);
  this->declare_parameter<float>("speed_scaling_max_output", speed_scaling_max_output_);
  this->get_parameter<float>("speed_scaling_max_output", speed_scaling_max_output_);
  this->declare_parameter<float>("upupup_speed", upupup_speed_);
  this->get_parameter<float>("upupup_speed", upupup_speed_);
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
  this->declare_parameter<float>("angle_ratio", angle_ratio_);
  this->get_parameter<float>("angle_ratio", angle_ratio_);
  this->declare_parameter<float>("angle_x_ratio", angle_x_ratio_);
  this->get_parameter<float>("angle_x_ratio", angle_x_ratio_);
  
  this->declare_parameter<float>("angle_x_pd_param", angle_x_pd_param_);
  this->get_parameter<float>("angle_x_pd_param", angle_x_pd_param_);

  this->declare_parameter<float>("find_line_speed", find_line_speed_);
  this->get_parameter<float>("find_line_speed", find_line_speed_);

  this->declare_parameter<int>("temp_scale", temp_scale_);
  this->get_parameter<int>("temp_scale", temp_scale_);

  /* 重要 BOOL 值 */
  this->declare_parameter<bool>("linkage_mode", linkage_mode);
  this->get_parameter<bool>("linkage_mode", linkage_mode);

  this->declare_parameter<bool>("start_cv", start_cv);
  this->get_parameter<bool>("start_cv", start_cv);

  point_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>("racing_track_center_detection", 10,
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1)); 

  target_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>("hobot_dnn_detection", 10,
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1)); 

  line_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("cx_cy", 10,
      std::bind(&RacingControlNode::subscription_callback_line, this, std::placeholders::_1)); 

  control_command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("sign4return", 10,
      std::bind(&RacingControlNode::control_command_callback_, this, std::placeholders::_1));

  patrol_command_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("start_car", 10,
      std::bind(&RacingControlNode::patrol_command_callback_, this, std::placeholders::_1));

  command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("command", 10,
      std::bind(&RacingControlNode::command_callback_, this, std::placeholders::_1));

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

  if (key_thread_ && key_thread_->joinable()) 
  {
    key_thread_->join();
    key_thread_ = nullptr;
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

//  {
//    std::unique_lock<std::mutex> lock(point_target_mutex_);
//    while (!line_queue_.empty())
//      line_queue_.pop();
//  }
}

void RacingControlNode::command_callback_(const std_msgs::msg::Int32::SharedPtr command_msg)
{
  if (command_msg->data == 2 or command_msg->data == 3)
  {
    rclcpp::Parameter param2("follow_linear_speed", upupup_speed_);
    this->set_parameter(param2);
  }
}

void RacingControlNode::subscription_callback_line(const std_msgs::msg::Int32MultiArray::SharedPtr line_msg)
{
  line_point[0] = line_msg->data[0];
  line_point[1] = line_msg->data[1];
  line_point[2] = line_msg->data[2];
  line_point[3] = line_msg->data[3];

  angle = line_msg->data[4];

//  {
//    std::unique_lock<std::mutex> lock(point_target_mutex_);
//    line_queue_.push(line_msg);
//    if (line_queue_.size() > 1) {
//      line_queue_.pop();
//    }
//  }
  return;
}

void RacingControlNode::checkKeyPress( void )
{
  std::cout << "=============================================================================" << std::endl;
  std::cout << "===================================START=====================================" << std::endl;
  std::cout << "=============================================================================" << std::endl;
  std::cout << "请按下启动键启动 Origincar......" << std::endl;

  while (true) 
  {
    if (start_car == true) 
    {
      start_patrol_process = true;
      std::cout << "已按下启动键!!!!!!" << std::endl;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay to avoid busy-waiting
  }
}

void RacingControlNode::control_command_callback_(const std_msgs::msg::Int32::SharedPtr msg)
{
  control_command = msg->data;
  if ( msg->data == 5 )
  {
    std::cout << "二维码识别成功，立即发布停车指令......" << std::endl;
    scan_qr_success = true;
  }
  else if ( msg->data == 6 )
  {
    rclcpp::Parameter param1("safe_distance", 170);
    this->set_parameter(param1);
    rclcpp::Parameter param2("follow_linear_speed", 0.5);
    this->set_parameter(param2);
  }

  return;
}

void RacingControlNode::patrol_command_callback_(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == true)
  {
    start_car = true;
  }
}

void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg)
{
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    point_queue_.push(point_msg);
    if (point_queue_.size() > 1) {
      point_queue_.pop();
    }
  }
  return;
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg)
{
  sub_target_ = true;

  for ( const auto &target : targets_msg->targets )
  {
    if ( target.rois[0].confidence > 0.75 )
    {
      if (target.type == "qr_code\r")
      {
        qr_vision.central_point[0] = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
        qr_vision.central_point[1] = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
        qr_vision.area = target.rois[0].rect.width * target.rois[0].rect.height;
      }
    }
    if (target.rois[0].confidence > 0.55)
    {
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
      // if (start_avoid != false)
      //   control_command = 6;
      // else
      // {
      //   std::cout << "避障流程已被打断!!!!!!" << std::endl;
      // }
      control_command = 6;
      start_avoid = false;
      start_find_line = true;
    }
  }

}

void RacingControlNode::MessageProcess()
{
  while(process_stop_ == false)
  {
    if (start_patrol_process)
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
      this->get_parameter<float>("avoid_ratio_coefficient_3", avoid_ratio_coefficient_3_);
      this->get_parameter<int>("safe_distance", safe_distance_);
      this->get_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
      this->get_parameter<float>("speed_scaling_factor", speed_scaling_factor_);
      this->get_parameter<float>("speed_scaling_max_output", speed_scaling_max_output_);
      this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
      this->get_parameter<int>("temp_scale", temp_scale_);
      this->get_parameter<float>("angle_ratio", angle_ratio_);
      this->get_parameter<float>("angle_x_ratio", angle_x_ratio_);
      this->get_parameter<float>("find_line_speed", find_line_speed_);
      this->get_parameter<float>("angle_x_pd_param", angle_x_pd_param_);

      this->get_parameter<float>("P_angle_param", pid_angle_.kp_);
      this->get_parameter<float>("D_angle_param", pid_angle_.kd_);
      this->get_parameter<float>("max_out_angle_param", pid_angle_.max_out_);
      this->get_parameter<float>("P_angle_x_param", pid_angle_x_.kp_);
      this->get_parameter<float>("D_angle_x_param", pid_angle_x_.kd_);
      this->get_parameter<float>("max_out_angle_x_param", pid_angle_x_.max_out_);
      this->get_parameter<float>("upupup_speed", upupup_speed_);
      /* 重要 BOOL 值 */
      this->get_parameter<bool>("linkage_mode", linkage_mode);
      this->get_parameter<bool>("start_cv", start_cv);

      std::unique_lock<std::mutex> lock(point_target_mutex_);
      //if (!point_queue_.empty() && sub_target_ == false && !line_queue_.empty() ){
      if (!point_queue_.empty() && sub_target_ == false){
        auto point_msg = point_queue_.top();
        //auto line_msg = line_queue_.top();

        lock.unlock();

        if ( control_command == 6 )
        {
          if (start_cv)
            ;
          else
            LineFollowing(point_msg->targets[0]);
          // LineFollowing(point_msg->targets[0]);
        }
        else if ( control_command == 5 )
          stop_car();

        lock.lock();
        point_queue_.pop();
       // line_queue_.pop();
      }
      //if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true && !line_queue_.empty()) 
      if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true) 
      {
        auto point_msg = point_queue_.top();
        //auto line_msg = line_queue_.top();
        auto targets_msg = targets_queue_.top();
        point_queue_.pop();
        targets_queue_.pop();
        //line_queue_.pop();
        lock.unlock();

        if ( control_command == 6 )
        {
          // std::cout << "targets_msg->targets.size() = " << targets_msg->targets.size() << std::endl;
          if (targets_msg->targets.size() == 0)
          {
              LineFollowing(point_msg->targets[0]);
          }
          else 
          {
            bool get_data = false;
            for(const auto &target : targets_msg->targets)
            {
              if (target.type == "obstacle\r")
              {
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                int distance = std::abs(320 - static_cast<int>(target.rois[0].rect.x_offset + target.rois[0].rect.width / 2));
                int area = target.rois[0].rect.width * target.rois[0].rect.height;

		int error = target.rois[0].rect.x_offset + target.rois[0].rect.width/2 - 320;

		if (error > 0)
		{
		  direction = "right";
		}
		else
		{
		  direction = "left";
		}

                if (bottom < bottom_threshold_ or area < obstacle_area_threshold_)
                {
                  if (get_data != true)
                    LineFollowing(point_msg->targets[0]);
                  get_data = true;
                }
                else 
                  if(target.rois[0].confidence > confidence_threshold_)
                  {
                    if (distance < safe_distance_)
                    {
                      control_command = 7;
                      // std::cout << "避障条件都满足!!!!!!" << std::endl;
                      std::cout << "area = " << area << std::endl;
                      std::cout << "distance = " << distance << std::endl;
                      std::cout << "bottom = " << bottom << std::endl;
                      std::cout << "离车最近的障碍物在线的：" << ori.type << std::endl;
		                  main_target_ = target;
                      start_avoid = true; // 打开避障
                      safe_distance_ = 0;
                      follow_linear_speed_ = 0.55;
                      rclcpp::Parameter param1("safe_distance", 0);
                      this->set_parameter(param1);
                      rclcpp::Parameter param2("follow_linear_speed", 0.55);
                      this->set_parameter(param2);
                      break;
                      // 让小车脱离寻线控制
                    }
                    else
                    {
                      std::cout << "distance = " << distance << std::endl;
                      std::cout << "该障碍物无需避过，可直接通过!!!!!!" << std::endl;
                    }
                  }
              }          
            }
            if (control_command == 6)
            {
              if (!get_data)
                  LineFollowing(point_msg->targets[0]);
            }
            
          }
        }
        else if (control_command == 5)
        {
          std::cout << "停车中......" << std::endl;
          stop_car();
          // start_trace = false;
          start_avoid = false;
          start_find_line = false;
          start_patrol_process = false;
          control_command = 7;
        }
          
        /* 跟踪控制 */
        if (start_trace)
        {
          if (scan_qr_success == true)
          {
            stop_car();
            std::cout << "关闭跟踪!!!!!!" << std::endl;
            std::cout << linkage_mode << std::endl;
            start_trace = false;
            scan_qr_success = false;
            if (linkage_mode)
            {
              std::cout << "关闭第一段寻线!!!!!!" << std::endl;
              start_patrol_process = false; // 关闭寻线线程
            }
          }
          if (start_trace)
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
          // if (start_avoid)
          // {
          //   publisher_->publish(avoid_twist_);
          //   std::cout << "正在避障......" << std::endl;
          // }
            
          if (start_trace)
          {
            publisher_->publish(trace_twist_);
            std::cout << "正在跟踪......" << std::endl;
          }
        }
        // std::cout << "control_command = " << control_command << std::endl;
        lock.lock();
      }
    }
  }
}

void RacingControlNode::stop_car(void)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  publisher_->publish(twist_msg);
}

void RacingControlNode::trace_central_point(void)
{
  int x = 0;
  int y = 0;

  x = int(qr_vision.central_point[0]);
  y = int(qr_vision.central_point[1]);

  float temp = x - 320.0;
  float angular_z = pid_.compute(temp);
  trace_twist_.linear.x = trace_linear_speed_;
  trace_twist_.angular.z = angular_z;
}

/* 避障画圆 */
void RacingControlNode::draw_circle(void)
{
  int temp = 1;
  if (ori.type == "left")
  {
    temp = 1;
    last_direction = "left";
  }
  else if (ori.type == "right")
  {
    temp = -1;
    last_direction = "right";
  }

  avoid_twist_.linear.x = avoid_linear_speed_;

  float angular_z = avoid_angular_ratio_; // 0.620455

  if (temp > 0)
    angular_z = angular_z;
  else
    angular_z = -angular_z;

  avoid_twist_.angular.z = angular_z;
  publisher_->publish(avoid_twist_);
  if (temp > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_1_));
  else
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_4_));

  if (temp > 0)
    avoid_twist_.angular.z = -angular_z * avoid_ratio_coefficient_3_;
  else
    avoid_twist_.angular.z = -angular_z * avoid_ratio_coefficient_1_;
  publisher_->publish(avoid_twist_);
  if (temp > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_2_));
  else
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_5_));

  // avoid_twist_.angular.z = angular_z * avoid_ratio_coefficient_2_;
  // if (temp > 0)
  //   std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_3_));
  // else
  //   std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_6_));
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &point_msg)
{
  std::cout << "RESNET巡线中......" << std::endl;
  int x = 0;
  x = int(point_msg.points[0].point[0].x);
  // std::cout << "x = " << x << std::endl;
    
  int y = int(point_msg.points[0].point[0].y) - 256;
  float temp = x - 320.0;
 if ((-10 < x ) && ( x < 0)) {
    temp = -10;
  } else if ((x > 0) && (x < 10)) {
    temp = 10;
  }

  if (start_find_line)
  {
    if (std::abs(angle) > temp_scale_ or std::abs((line_point[0] + line_point[2])/2 - 320) > 50)
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = find_line_speed_;
      //twist_msg.angular.z = angle * angle_ratio_ + ((line_point[0] + line_point[2])/2 - 320) * angle_x_ratio_;
      
      twist_msg.angular.z = pid_angle_.compute(-angle) + pid_angle_x_.compute(((line_point[0] + line_point[2])/2 - 320));
      std::cout << "angle = " << angle << std::endl;
      // std::cout << "temp_scale_ = " << temp_scale_ << std::endl;

      std::cout << "正在找线" << std::endl;
      publisher_->publish(twist_msg);
    }
    else
    {
      start_find_line = false;
    }

    return; 
  }



  auto twist_msg = geometry_msgs::msg::Twist();
  // float angular_z = follow_angular_ratio_ * temp / 150.0  * y / 224.0;

  // auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = 0.0;
  // if (std::abs(temp > 5))
  // {
    angular_z = pid_.compute(temp);
  // }
  // else
  //   angular_z = follow_angular_ratio_ * 5 / 150.0  * y / 224.0;
  float value = std::abs(temp) * speed_scaling_factor_;
  if ( value > speed_scaling_max_output_ )
    value = speed_scaling_max_output_;

  if (std::abs(temp) == 10)
  {
    value = 0.0;
  }

  twist_msg.linear.x = follow_linear_speed_ - value;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);

  //std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void RacingControlNode::LineFollow(const std_msgs::msg::Int32MultiArray &line_msg)
{
  std::cout << "CV巡线中......" << std::endl;
  // int x = 0;
  // if (start_cv)
  // {
  //   x = line_point[0];
  //   std::cout << "x = " << x << std::endl;
  // }
  // else
  // {
  //   x = int(point_msg.points[0].point[0].x);
  //   std::cout << "x = " << x << std::endl;
  // }

  // x = line_point[0];
  // std::cout << "x = " << x << std::endl; 

  int x = int(line_msg.data[0]);
  int y = int(line_msg.data[0]);

  if (y < 400)
    x = 0.4 * x + 0.6 * last_x;
  std::cout << "x = " << x << std::endl;
  float temp = x - 320.0;

  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = pid_.compute(temp);
  float value = std::abs(temp) * speed_scaling_factor_;
  if ( value > speed_scaling_max_output_ )
    value = speed_scaling_max_output_;

  twist_msg.linear.x = follow_linear_speed_ - value;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);

  last_x = x;
}

int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate_1"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
