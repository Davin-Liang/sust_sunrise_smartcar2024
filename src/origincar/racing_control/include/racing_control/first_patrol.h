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

#ifndef FIRST_PATROL_H_
#define FIRST_PATROL_H_

#include <vector>
#include <queue>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp" // own
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/target.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

struct QrVisionInfo
{
  /* data */
  float central_point[2] = {0.0, 0.0};
  float area = 0.0;

  void print() const
  {
    std::cout << "centralpoint" << central_point[0] << central_point[1] << std::endl;
    std::cout << "area" << area << std::endl;
  }  
};

struct obstacle_ori
{
  /* data */
  float area = 0.0;
  std::string type = "";
};


class PIDController
{
public:
    PIDController()
    {
      ;
    }

    float compute(float current_position)
    {
        float error = target_position_ - current_position;
        integral_ += error;

        // 限制积分项
        if (integral_ > integral_limit_)
            integral_ = integral_limit_;
        else if (integral_ < -integral_limit_)
            integral_ = -integral_limit_;

        float derivative = error - prev_error_;
        float control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;

        if (control_signal > max_out_)
          control_signal = max_out_;
        else if (control_signal < -max_out_)
          control_signal = - max_out_;

        return control_signal;
    }

    void setTargetPosition(float target_position)
    {
        target_position_ = target_position;
    }
    
    float kp_               = 0.004;
    float ki_               = 0.0;
    float kd_               = 0.01;
    float target_position_  = 0.0;
    float prev_error_       = 0.0;
    float integral_         = 0.0;
    float integral_limit_   = 0.0; // 积分限幅
    float max_out_          = 1.0;

private:

};

class RacingControlNode : public rclcpp::Node{
public:
  RacingControlNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void trace_central_point(void);
  
  ~RacingControlNode() override;
  
  PIDController pid_;
  PIDController pid_angle_;
  PIDController pid_angle_x_;
  obstacle_ori ori;
  
private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr line_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr patrol_command_subscriber_; // own
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_command_subscriber_; // own
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg);
    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void subscription_callback_line(const std_msgs::msg::Int32MultiArray::SharedPtr line_msg);
    void control_command_callback_(const std_msgs::msg::Int32::SharedPtr msg);
    void command_callback_(const std_msgs::msg::Int32::SharedPtr command_msg);
    void patrol_command_callback_(const std_msgs::msg::Bool::SharedPtr msg);
    void stop_car(void); // own
    void draw_circle(void);

    void LineFollowing(const ai_msgs::msg::Target &point_msg);
    void LineFollow(const std_msgs::msg::Int32MultiArray &line_msg);
    void MessageProcess(void);
    void avoid_obstacle(void);
    void checkKeyPress( void );
    std::string pub_control_topic_ = "cmd_vel";

    std::priority_queue<std_msgs::msg::Int32MultiArray::SharedPtr,
                      std::vector<std_msgs::msg::Int32MultiArray::SharedPtr>>
    line_queue_;

    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>>
    point_queue_;

    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>>
    targets_queue_;

    std::mutex point_target_mutex_;
    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;
    std::shared_ptr<std::thread> avoid_thread_;
    std::shared_ptr<std::thread> key_thread_;

    float speed_scaling_factor_       = 0.01;
    float speed_scaling_max_output_   = 0.1;
    float avoid_angular_ratio_        = 0.620455;
    float avoid_linear_speed_         = 0.18;
    float follow_linear_speed_        = 0.3;
    float trace_linear_speed_         = 0.03;
    float avoid_time_1_               = 1.75;
    float avoid_time_2_               = 1.75;
    float avoid_time_3_               = 1.0;
    float avoid_time_4_               = 1.75;
    float avoid_time_5_               = 1.75;
    float avoid_time_6_               = 1.0;
    float avoid_ratio_coefficient_1_  = 1.5;
    float avoid_ratio_coefficient_2_  = 1.0;
    float avoid_ratio_coefficient_3_  = 1.0;
    int safe_distance_                = 100;
    int bottom_threshold_             = 241;
    float confidence_threshold_       = 0.75;
    int qr_area_threshold_            = 5000;
    int obstacle_area_threshold_      = 2000;
    float follow_angular_ratio_       = 0.1;
    float upupup_speed_               = 0.9;

    int temp_scale_                   = 56;

    int angle = 0;

    std::string direction = "left";

    float angle_ratio_                = 0.1;
    float angle_x_ratio_              = 0.1;
    float find_line_speed_            = 0.1;
    float angle_x_pd_param_           = 0.1; 

    int line_point[4] = {0, 0, 0, 0};
    int last_x = 0;

    std::string last_direction = "left";


    bool sub_target_                  = false; 
    bool start_avoid                  = false;
    bool start_trace                  = false;
    bool start_detect                 = true;
    bool scan_qr_success              = false;
    bool linkage_mode                 = false;
    bool start_car                    = false;
    bool start_patrol_process         = false;
    bool start_cv                     = false;
    bool start_find_line              = false;

    int32_t control_command           = 6; // default number is 5 that reperesents stoping car.
    ai_msgs::msg::Target main_target_;
    geometry_msgs::msg::Twist avoid_twist_;
    geometry_msgs::msg::Twist trace_twist_;

    QrVisionInfo qr_vision;
};


#endif  // FIRST_PATROL_H_
