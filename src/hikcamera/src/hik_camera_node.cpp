#include "hik_camera_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>//异常报错
namespace ne_io
{
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node",options),
  hk_cam_(),  
  image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("ne_io__camera/image_raw", 2)) 
{
    RCLCPP_INFO(this->get_logger(), "节点已经启动.");

    this->declare_parameter<double>("exposuretime",3000.00);
    this->declare_parameter<double>("gain",8.00);//注意这个参数不能调成0或115，在这个范围内调，选好gain改变exposuretime最好
    
    // 添加参数更改回调
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto &param : params) {
                    RCLCPP_INFO(this->get_logger(),"更新参数%s=%f",param.get_name().c_str(),param.as_double());
                    if (param.get_name() == "exposuretime") {
                        this->newexposuretime = param.as_double();
                        hk_cam_.reNewExposureStart(newexposuretime);
                    }else if(param.get_name() == "gain"){
                        this->newgain = param.as_double();
                        hk_cam_.reNewGainStart(newgain);
                    }
                }
                return result;
            }
    );
   
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&HikCameraNode::hikImgCallback, this)
    );
    hk_cam_.start();
}

void HikCameraNode::hikImgCallback()
{
   
    try{
    imgChange();
   }catch (const cv::Exception& e){
    RCLCPP_INFO(this->get_logger(), "Failed to convert Bayer image to RGB");
    hk_cam_.reConnectStart();
    std::cout<<"yessssss!!!!"<<std::endl;
    imgChange();
   }
}

HikCameraNode::~HikCameraNode(){
    if(cam_thread_.joinable()){
        cam_thread_.join();
    }
}

void HikCameraNode::imgChange(){
    hk_cam_.getImg();
    cv::Mat src(cv::Size(hk_cam_.stImageInfo.stFrameInfo.nWidth, hk_cam_.stImageInfo.stFrameInfo.nHeight), CV_8UC1);
    std::memcpy(src.data, hk_cam_.stImageInfo.pBufAddr, hk_cam_.stImageInfo.stFrameInfo.nWidth * hk_cam_.stImageInfo.stFrameInfo.nHeight);
    //RCLCPP_INFO(this->get_logger(), "Source image: Width=%d, Height=%d, Channels=%d",
            // src.cols, src.rows, src.channels());
    cv::Mat bgr_image;
    cv::cvtColor(src, bgr_image, cv::COLOR_BayerRG2RGB); 

    img_msg_header_.frame_id = "hikcamera_node";
    img_msg_header_.stamp = now();
    auto msg = cv_bridge::CvImage(img_msg_header_, "bgr8", bgr_image).toImageMsg();

    image_publisher_->publish(*msg);  // 发布图像消息
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ne_io::HikCameraNode)