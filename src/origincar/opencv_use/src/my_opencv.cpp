#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageToCompressedImageNode : public rclcpp::Node
{
public:
    ImageToCompressedImageNode()
        : Node("image_to_compressed_image_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10,
            std::bind(&ImageToCompressedImageNode::imageCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_image_topic", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // ֱ�ӽ� Image ��Ϣ�е�������Ϊ CompressedImage ��Ϣ����
            auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
            compressed_msg->format = "jpeg";  // ����Ϊ JPEG ��ʽ�����Ը�����Ҫ�޸�

            // ��ԭʼͼ�����ݿ����� CompressedImage ��Ϣ��
            compressed_msg->data.assign(msg->data.begin(), msg->data.end());

            publisher_->publish(*compressed_msg);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception thrown while processing image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageToCompressedImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;}