#ifndef HIK_CAMERA_NODE_HPP
#define HIK_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>
#include <string>

#include "hik_camera_io.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace ne_io
{

class ImagePublisherNode : public rclcpp::Node
{
public:
   ImagePublisherNode(const rclcpp::NodeOptions & options);
    ~ImagePublisherNode();

private:
    void hikImgCallback();
    std::unique_ptr<cv::Mat> src;
    HikCam hk_cam_; 
    std::thread cam_thread_;
    double my_param;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; 
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}//namespace ne_io

#endif 
