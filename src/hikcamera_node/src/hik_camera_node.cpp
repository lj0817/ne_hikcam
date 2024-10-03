#include "hik_camera_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
namespace ne_io
{

using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

ImagePublisher::ImagePublisher(const std::string &name)
: Node(name),
  hk_cam_(),  
  image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10)) 
{
    RCLCPP_INFO(this->get_logger(), "%s 节点已经启动.", name.c_str());

    this->declare_parameter<double>("exposuretime",3000.00);
    
    // 添加参数更改回调
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto &param : params) {
                    RCLCPP_INFO(this->get_logger(),"更新参数%s=%f",param.get_name().c_str(),param.as_double());
                    if (param.get_name() == "exposuretime") {
                        
                        this->my_param = param.as_double();
                        hk_cam_.reStart(this->my_param);
                    }
                }
                return result;
            }
    );
   
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ImagePublisher::hikImgCallback, this)
    );
    hk_cam_.start();
}

void ImagePublisher::hikImgCallback()
{
   
    hk_cam_.getImg();
    cv::Mat src(cv::Size(hk_cam_.stImageInfo.stFrameInfo.nWidth, hk_cam_.stImageInfo.stFrameInfo.nHeight), CV_8UC1);
    std::memcpy(src.data, hk_cam_.stImageInfo.pBufAddr, hk_cam_.stImageInfo.stFrameInfo.nWidth * hk_cam_.stImageInfo.stFrameInfo.nHeight);
    //RCLCPP_INFO(this->get_logger(), "Source image: Width=%d, Height=%d, Channels=%d",
            // src.cols, src.rows, src.channels());
    cv::Mat bgr_image;
    cv::cvtColor(src, bgr_image, cv::COLOR_BayerGR2BGR,3); 
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
    image_publisher_->publish(*msg);  // 发布图像消息
}

ImagePublisher::~ImagePublisher(){
    if(cam_thread_.joinable()){
        cam_thread_.join();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>("camera_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

}namespace ne_io
