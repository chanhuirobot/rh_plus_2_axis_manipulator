#ifndef USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_
#define USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_

#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>

namespace usb_camera_driver
{

class CameraDriver : public rclcpp::Node {
public:
    // explicit 키워드는 자신이 원하지 않은 형변환이 일어나지 않도록 제한하는 키워드
    // 생성자와 소멸자
    explicit CameraDriver(const rclcpp::NodeOptions&);
    ~CameraDriver() {};

private:
    // 각각의 변수 선언
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;

    bool is_flipped;

    std::string frame_id_;
    int image_height_;
    int image_width_;
    double fps_;
    int camera_id;

    std::chrono::steady_clock::time_point last_frame_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;

    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;

    // 함수 선언. 근데 왜 대문자 시작하지? 대문자 시작하면 클래스 아닌가? 허허
    std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame);

    void ImageCallback();
};

}

#endif // USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_
