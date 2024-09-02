#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class LineFollowerNode : public rclcpp::Node {
public:
    LineFollowerNode() : Node("line_follower") {
        // 声明与颜色提取相关的参数及其默认值
        auto pd_black_hue_min = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_black_hue_max = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_black_saturation_min = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_black_saturation_max = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_black_value_min = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_black_value_max = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();

        auto pd_search_top = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
        auto pd_search_bot = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();

        pd_black_hue_min->description = "Black Hue Min";
        pd_black_hue_max->description = "Black Hue Max";
        pd_black_saturation_min->description = "Black Saturation Min";
        pd_black_saturation_max->description = "Black Saturation Max";
        pd_black_value_min->description = "Black Value Min";
        pd_black_value_max->description = "Black Value Max";
        pd_search_top->description = "Search Top";
        pd_search_bot->description = "Search Bot";

        auto range_hue = std::make_shared<rcl_interfaces::msg::IntegerRange>();
        range_hue->from_value = 0;
        range_hue->to_value = 180;
        range_hue->step = 1;

        auto range_saturation_value = std::make_shared<rcl_interfaces::msg::IntegerRange>();
        range_saturation_value->from_value = 0;
        range_saturation_value->to_value = 255;
        range_saturation_value->step = 1;

        pd_black_hue_min->integer_range.emplace_back(*range_hue);
        pd_black_hue_max->integer_range.emplace_back(*range_hue);
        pd_black_saturation_min->integer_range.emplace_back(*range_saturation_value);
        pd_black_saturation_max->integer_range.emplace_back(*range_saturation_value);
        pd_black_value_min->integer_range.emplace_back(*range_saturation_value);
        pd_black_value_max->integer_range.emplace_back(*range_saturation_value);

        range_hue->from_value = 0;
        range_hue->to_value = 480;
        range_hue->step = 1;

        pd_search_top->integer_range.emplace_back(*range_hue); // 使用 hue 的范围作为示例
        pd_search_bot->integer_range.emplace_back(*range_hue);

        this->declare_parameter("black_hue_min", 0, *pd_black_hue_min);
        this->declare_parameter("black_hue_max", 180, *pd_black_hue_max);
        this->declare_parameter("black_saturation_min", 0, *pd_black_saturation_min);
        this->declare_parameter("black_saturation_max", 255, *pd_black_saturation_max);
        this->declare_parameter("black_value_min", 0, *pd_black_value_min);
        this->declare_parameter("black_value_max", 50, *pd_black_value_max);
        this->declare_parameter("search_top", 360, *pd_search_top); // 示例值
        this->declare_parameter("search_bot", 480, *pd_search_bot); // 示例值
        
        // 获取参数值
        this->get_parameter("black_hue_min", black_hue_min_);
        this->get_parameter("black_hue_max", black_hue_max_);
        this->get_parameter("black_saturation_min", black_saturation_min_);
        this->get_parameter("black_saturation_max", black_saturation_max_);
        this->get_parameter("black_value_min", black_value_min_);
        this->get_parameter("black_value_max", black_value_max_);
        this->get_parameter("search_top", search_top_);
        this->get_parameter("search_bot", search_bot_);
        
        // 创建图像订阅者
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&LineFollowerNode::imageCallback, this, std::placeholders::_1));
        
        // 创建发布者
        cx_cy_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("cx_cy", 10);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        
        // 创建参数回调
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&LineFollowerNode::parameterCallback, this, std::placeholders::_1));
    }

private:
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters) {
        for (const auto &param : parameters) {
            if (param.get_name() == "black_hue_min") {
                black_hue_min_ = param.as_int();
            } else if (param.get_name() == "black_hue_max") {
                black_hue_max_ = param.as_int();
            } else if (param.get_name() == "black_saturation_min") {
                black_saturation_min_ = param.as_int();
            } else if (param.get_name() == "black_saturation_max") {
                black_saturation_max_ = param.as_int();
            } else if (param.get_name() == "black_value_min") {
                black_value_min_ = param.as_int();
            } else if (param.get_name() == "black_value_max") {
                black_value_max_ = param.as_int();
            } else if (param.get_name() == "search_top") {
                search_top_ = param.as_int();
            } else if (param.get_name() == "search_bot") {
                search_bot_ = param.as_int();
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::cout << "Received an image" << std::endl;

        // 检查图像的编码格式
        if (msg->encoding != "bgr8" && msg->encoding != "rgb8") {
            std::cerr << "Unsupported encoding format" << std::endl;
            return;
        }

        // 将 ROS 图像消息转换为 OpenCV 的 cv::Mat
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;

        // 检查转换是否成功
        if (image.empty()) {
            std::cerr << "Failed to convert image" << std::endl;
            return;
        }

        // 输出转换成功信息
        std::cout << "Image conversion successful" << std::endl;

        // 高斯模糊平滑图像，固定模糊核大小
        int blur_size = 5; // 高斯模糊核大小（必须为奇数）
        if (blur_size % 2 == 0) blur_size++; // 确保模糊核大小为奇数
        cv::Mat blurred;
        cv::GaussianBlur(image, blurred, cv::Size(blur_size, blur_size), 0);

        // 将图像转换为 HSV 颜色空间
        cv::Mat hsv;
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        // 基于 ROS 参数的动态阈值分割
        cv::Mat mask;
        cv::inRange(hsv,
                    cv::Scalar(black_hue_min_, black_saturation_min_, black_value_min_),
                    cv::Scalar(black_hue_max_, black_saturation_max_, black_value_max_),
                    mask);

        // 形态学操作，固定结构元素大小
        int morph_element_size = 5; // 形态学操作的结构元素大小
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_element_size, morph_element_size));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 使用 Canny 边缘检测，固定阈值
        int canny_low_threshold = 50;  // Canny 边缘检测的低阈值
        int canny_high_threshold = 150; // Canny 边缘检测的高阈值
        cv::Mat edges;
        cv::Canny(mask, edges, canny_low_threshold, canny_high_threshold);

        // 定义搜索区域
        int h = image.rows;
        int w = image.cols;

        // 在搜索区域外部清除边缘检测结果
        for (int i = 0; i < search_top_; i++) {
            for (int j = 0; j < w; j++) {
                edges.at<uchar>(i, j) = 0;
            }
        }
        for (int i = search_bot_; i < h; i++) {
            for (int j = 0; j < w; j++) {
                edges.at<uchar>(i, j) = 0;
            }
        }

        // 计算图像的矩
        cv::Moments M = cv::moments(edges);

        std_msgs::msg::Int32MultiArray msg_out;
        if (M.m00 > 0) {
            int cx = int(cvRound(M.m10 / M.m00));
            int cy = int(cvRound(M.m01 / M.m00));

            // 创建一个新的图像来显示点
            cv::Mat result_image = mask.clone(); // 使用 mask 作为背景图像
            cv::cvtColor(result_image, result_image, cv::COLOR_GRAY2BGR); // 将灰度图像转换为彩色图像

            // 在处理后的图像中绘制中心点
            cv::circle(result_image, cv::Point(cx, cy), 15, cv::Scalar(0, 0, 255), -1);

            // 设置消息数据
            msg_out.data.push_back(cx);
            msg_out.data.push_back(cy);
            cx_cy_pub_->publish(msg_out);

            // 将 result_image 发布为压缩图像消息
            auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_image).toImageMsg();
            image_pub_->publish(*processed_msg);

        } else {
            RCLCPP_INFO(this->get_logger(), "未找到线条!");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cx_cy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    // 颜色提取相关参数
    int black_hue_min_;              // 黑色的最小色调
    int black_hue_max_;              // 黑色的最大色调
    int black_saturation_min_;       // 黑色的最小饱和度
    int black_saturation_max_;       // 黑色的最大饱和度
    int black_value_min_;            // 黑色的最小亮度
    int black_value_max_;            // 黑色的最大亮度

    // 搜索区域相关参数
    int search_top_;                 // 搜索区域的顶部
    int search_bot_;                 // 搜索区域的底部
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
