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
namespace ne_io{

class HikCameraNode : public rclcpp::Node
{
public:
   HikCameraNode(const rclcpp::NodeOptions & options);
    ~HikCameraNode();

private:
    bool hikImgCallback();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; 
    std::unique_ptr<cv::Mat> src;
    HikCam hk_cam_; 
    std::thread cam_thread_;
    double my_param;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}

#endif 
