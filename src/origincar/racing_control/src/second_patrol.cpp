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

#include "racing_control/second_patrol.h"

#include <unistd.h>
#include <chrono> // 包含 std::chrono::seconds
#include <iostream>
#include <stdlib.h>


RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options)
{
  if (!msg_process_)
    msg_process_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::MessageProcess, this));

  if ( !avoid_thread_ )
    avoid_thread_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::avoid_obstacle, this));

  if ( !time_thread_ )
    time_thread_ = std::make_shared<std::thread>(std::bind(&RacingControlNode::count_time, this));
  
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
  this->declare_parameter<float>("start_avoid_time", start_avoid_time_);
  this->get_parameter<float>("start_avoid_time", start_avoid_time_);
  /* 跟踪参数 */
  this->declare_parameter<float>("trace_linear_speed", trace_linear_speed_);
  this->get_parameter<float>("trace_linear_speed", trace_linear_speed_);
  this->declare_parameter<float>("confidence_threshold", confidence_threshold_);
  this->get_parameter<float>("confidence_threshold", confidence_threshold_);

  this->declare_parameter<bool>("linkage_mode", linkage_mode);
  this->get_parameter<bool>("linkage_mode", linkage_mode);


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

  if (linkage_thread_ && linkage_thread_->joinable()) 
  {
    linkage_thread_->join();
    linkage_thread_ = nullptr;
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

void RacingControlNode::count_time( void )
{
  while ( 1 )
  {
    if (start_count_time)
    {
      safe_distance_ = 0;
      rclcpp::Parameter param1("safe_distance", 0);
      this->set_parameter(param1); 
      std::this_thread::sleep_for(std::chrono::duration<double>(start_avoid_time_));
      safe_distance_ = 200;
      rclcpp::Parameter param2("safe_distance", 200);
      this->set_parameter(param2);

      start_count_time = false;      
    }
  }

  std::this_thread::sleep_for(std::chrono::duration<double>(start_avoid_time_));
  rclcpp::Parameter param1("safe_distance", 200);
  this->set_parameter(param1);
}

void RacingControlNode::linkage_adjust(void)
{ 
  while (true)
  {
    if (linkage_mode)
    {
      // 联动模式已开启
      start_patrol_process = false;
      if (step == 0)
        if (control_command == 5)
          step = 1;
      if (step == 1)
        if (control_command == 6)
        {
          start_patrol_process = true;
          break;
        }
    }
    else
    {
      start_patrol_process = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void RacingControlNode::control_command_callback_(const std_msgs::msg::Int32::SharedPtr msg)
{
  control_command = msg->data;

  if (msg->data == 6)
  {
    start_count_time = true;
  }

  if (linkage_mode)
  {
    start_patrol_process = false;
    if (step == 0)
      if (msg->data == 5)
        step = 1;
    if (step == 1)
      if (msg->data == 6)
      {
        std::cout << "第二段寻线已开启!!!!!!" << std::endl;
        start_patrol_process = true;
      }
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

  if (!linkage_mode)
    start_patrol_process = true;

  return;
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg)
{
  sub_target_ = true;

  for ( const auto &target : targets_msg->targets )
  {
    if ( target.rois[0].confidence > 0.8 )
    {
      if (target.type == "parking_sign\r")
      {
        home_vision.central_point[0] = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
        home_vision.central_point[1] = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
        home_vision.area = target.rois[0].rect.width * target.rois[0].rect.height;

        control_command = 7;
        start_home = true;
        start_avoid = false;
        std::cout << "打断!!!" << std::endl;
        // stopRequested.store(true);
        // cv.notify_one(); // 通知worker线程中断
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
      if (start_avoid != false)
        control_command = 6;
      else
      {
        std::cout << "避障流程已被打断!!!!!!" << std::endl;
      }
      start_avoid = false;

      // stopRequested.store(false);
      // // stopRequested.store(false);
      // cv.notify_one(); // 通知worker线程中断
    }
  }

}

void RacingControlNode::MessageProcess(){

  
  while ( process_stop_ == false )
  {
    if(start_patrol_process)
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
      this->get_parameter<float>("avoid_ratio_coefficient_1", avoid_ratio_coefficient_1_);
      this->get_parameter<float>("avoid_ratio_coefficient_2", avoid_ratio_coefficient_2_);
      this->get_parameter<int>("safe_distance", safe_distance_);
      this->get_parameter<int>("obstacle_area_threshold", obstacle_area_threshold_);
      this->get_parameter<bool>("linkage_mode", linkage_mode);
      this->get_parameter<float>("speed_scaling_factor", speed_scaling_factor_);
      this->get_parameter<float>("speed_scaling_max_output", speed_scaling_max_output_);
      this->get_parameter<float>("avoid_ratio_coefficient_3", avoid_ratio_coefficient_3_);
      this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
      this->get_parameter<float>("start_avoid_time", start_avoid_time_);

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
            bool get_data = false;
            for(const auto &target : targets_msg->targets)
            {
              if ( target.type == "obstacle\r" )
              {
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                int distance = std::abs(320 - static_cast<int>(target.rois[0].rect.x_offset + target.rois[0].rect.width / 2));
                int area = target.rois[0].rect.width * target.rois[0].rect.height;
                if (bottom < bottom_threshold_ or area < obstacle_area_threshold_)
                {
                  if (get_data != true)
                    LineFollowing(point_msg->targets[0]);
                  get_data = true;
                }
                else 
                  if( target.rois[0].confidence > confidence_threshold_ )
                  {
                    if ( distance < safe_distance_ )
                    
                    {
                      control_command = 7;
                      std::cout << "area = " << area << std::endl;
                      std::cout << "distance = " << distance << std::endl;
                      std::cout << "bottom = " << bottom << std::endl;
                      std::cout << "离车最近的障碍物在线的：" << ori.type << std::endl;
		                  main_target_ = target;
                      start_avoid = true; // 打开避障

                      break;
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
          start_home = false;
          start_avoid = false;
          control_command = 7;
        }

        if (start_home)
        {
          control_command = 7;
          trace_central_point();
        }

        if (control_command != 5)
        {
          // if (start_avoid)
          // {
          //   publisher_->publish(avoid_twist_);
          //   std::cout << "正在避障......" << std::endl;
          // }
            
          if (start_home)
          {
            publisher_->publish(trace_twist_);
            std::cout << "正在跟踪......" << std::endl;
          }
        }

        lock.lock();
      }
    }
  }
}

void RacingControlNode::trace_central_point(void)
{
  int x = 0;
  int y = 0;

  x = int(home_vision.central_point[0]);
  y = int(home_vision.central_point[1]);

  float temp = x - 320.0;
  float angular_z = pid_.compute(temp);
  trace_twist_.linear.x = trace_linear_speed_;
  trace_twist_.angular.z = angular_z;
}

void RacingControlNode::stop_car(void)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  publisher_->publish(twist_msg);
}

/* 避障画圆 */
void RacingControlNode::draw_circle(void)
{
  std::unique_lock<std::mutex> lock(mtx);
  
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
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_2_));
    // cv.wait_for(lock, std::chrono::duration<double>(avoid_time_2_), []() { return stopRequested.load(); });
    // std::cout << "stopRequested before wait: " << stopRequested.load() << std::endl;
    // cv.wait_for(lock, std::chrono::duration<double>(avoid_time_2_), [this]() { return stopRequested.load(); });

    // if (stopRequested.load()) 
    // {
    //   std::cout << "Operation interrupted!" << std::endl;
    // } else 
    // {
    //   std::cout << "Operation completed!" << std::endl;
    // }
  }
  else
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_5_));
    // cv.wait_for(lock, std::chrono::duration<double>(avoid_time_5_), []() { return stopRequested.load(); });
    // std::cout << "stopRequested before wait: " << stopRequested.load() << std::endl;
    // cv.wait_for(lock, std::chrono::duration<double>(avoid_time_5_), [this]() { return stopRequested.load(); });


    // if (stopRequested.load()) 
    // {
    //   std::cout << "Operation interrupted!" << std::endl;
    // } 
    // else 
    // {
    //   std::cout << "Operation completed!" << std::endl;
    // }
  }
    

 // avoid_twist_.angular.z = angular_z * avoid_ratio_coefficient_2_;
 // publisher_->publish(avoid_twist_);
  //if (temp > 0)
 // {
  //  std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_3_));
   
    // cv.wait_for(lock, std::chrono::seconds(avoid_time_3_), []() { return stopRequested.load(); });

    // if (stopRequested.load()) 
    // {
    //   std::cout << "Operation interrupted!" << std::endl;
    // } else 
    // {
    //   std::cout << "Operation completed!" << std::endl;
    // }


 // }
//  else
 // {
    // cv.wait_for(lock, std::chrono::seconds(avoid_time_6_), []() { return stopRequested.load(); });

    // if (stopRequested.load()) 
    // {
    //   std::cout << "Operation interrupted!" << std::endl;
    // } else 
    // {
    //   std::cout << "Operation completed!" << std::endl;
    // }
   // std::this_thread::sleep_for(std::chrono::duration<double>(avoid_time_6_));
 // }  
    
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &point_msg)
{
  std::cout << "RESNET巡线中......" << std::endl;
  int x = 0;
  x = int(point_msg.points[0].point[0].x);
  // int y = int(point_msg.points[0].point[0].y) - 256;
  float temp = x - 320.0;
  if ((-10 < x ) && ( x < 0)) {
    temp = -10;
  } else if ((x > 0) && (x < 10)) {
    temp = 10;
  }
  
  
  auto twist_msg = geometry_msgs::msg::Twist();
  // float angular_z = follow_angular_ratio_ * temp / 150.0  * y / 224.0;

  // auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = 0.0;
  angular_z = pid_.compute(temp);
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

  }
  
  
  int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate_2"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
