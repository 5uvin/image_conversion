#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>


using std::placeholders::_1; 

bool gray = 0;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("change_mode_server") {

    srv_ = create_service<std_srvs::srv::SetBool>(
        "change_mode", std::bind(&ServerNode::service_callback, this,
                                std::placeholders::_1, std::placeholders::_2));
    // publisher_ =
    //     this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void service_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
  {
    RCLCPP_INFO(this->get_logger(), "Requested Data: %d",
                request->data);
    gray = request->data;
    RCLCPP_INFO(this->get_logger(), "Global Variable Data: %d",
            gray);
    // auto message = geometry_msgs::msg::Twist();
    // if (request->data == 0) {
    //   message.linear.x = 0.25;
    //   message.angular.z = -0.25;
    //   response->message = "Robot Moving";
    // } else {
    //   message.linear.x = 0.0;
    //   message.angular.z = 0.0;
    //   response->message = "Robot Stopped";
    // }
    // publisher_->publish(message);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                response->message.c_str());
  }
};

class ImageRepublisher : public rclcpp::Node
{
public:
    ImageRepublisher() : Node("image_conversion_node")
    {   
        this->declare_parameter<std::string>("input_camera_topic", "/image_raw");
        this->declare_parameter<std::string>("output_camera_topic", "/image_repub");

        input_camera_topic_ = this->get_parameter("input_camera_topic").as_string();
        output_camera_topic_ = this->get_parameter("output_camera_topic").as_string();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(input_camera_topic_, 10, std::bind(&ImageRepublisher::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_camera_topic_, 10);
    }

    void topic_callback(const sensor_msgs::msg::Image &msg) const
    {
        if (gray == 0)
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::YUV422_YUY2);

            cv::Mat grayMat;
            cv::cvtColor(cv_ptr->image, grayMat, cv::COLOR_YUV2GRAY_YUY2);

            auto gray_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", grayMat).toImageMsg();

            // Publish grayscale image
            publisher_->publish(*gray_msg);            
        } else
        {
            this->publisher_->publish(msg);
        }
        // Write a message every time a new message is received on the topic.
        // RCLCPP_INFO(this->get_logger(), "Global Variable Data: %d",
        // gray);
        // this->publisher_->publish(msg);
        // RCLCPP_INFO_STREAM(get_logger(), "I heard: ");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    sensor_msgs::msg::Image::SharedPtr gray_msg;

    std::string input_camera_topic_;
    std::string output_camera_topic_;

};

int main(int argc, char * argv[])
{
 
  // Initialize ROS 2.
  rclcpp::init(argc, argv);
 
  // Create an instance of the MinimalCppSubscriber node and keep it running.
  auto image_conversion_node = std::make_shared<ImageRepublisher>();
  auto service_node = std::make_shared<ServerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(image_conversion_node);
  executor.add_node(service_node);
  executor.spin();
 
  // Shutdown ROS 2 upon node termination.
  rclcpp::shutdown();
 
  // End of program.
  return 0;
}