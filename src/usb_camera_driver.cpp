#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rh_plus_2_axis_manipulator/usb_camera_driver.hpp"

using namespace std::chrono_literals;

// 생성자 구현
CameraDriver::CameraDriver(const rclcpp::NodeOptions &node_options) : Node("usb_camera_driver", node_options)
{
    // 파라미터 선언
    frame_id_ = this->declare_parameter("frame_id", "camera");
    // parameter 이름 : my_parameter, 기본값 world
    image_width_ = this->declare_parameter("image_width", 640);
    image_height_ = this->declare_parameter("image_height", 480);
    fps_ = this->declare_parameter("fps", 10.0);

    camera_id = this->declare_parameter("camera_id", 0);

    // qos 설정
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    // 퍼블리셔 설정
    camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    /* get ROS2 config parameter for camera calibration file */
    auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);

    // 비디오 캡쳐 객체 다루기
    cap.open(camera_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

    last_frame_ = std::chrono::steady_clock::now();

    timer_ = this->create_wall_timer(1ms, std::bind(&CameraDriver::ImageCallback, this));
}

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::ConvertFrameToMessage(cv::Mat &frame)
{
    // 변수 선언
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // Make sure output in the size the user wants even if it is not native
    if(frame.rows != image_width_ || frame.cols != image_height_){
        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
    }

    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8";
    /* FIXME c++20 has std::endian */
    // ros_image.is_bigendian = (std::endian::native == std::endian::big);
    ros_image.is_bigendian = false;
    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    if (frame.isContinuous())
    {
        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
    }
    else
    {
        // copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
        uchar *cv_data_ptr = frame.data;
        for(int i = 0; i < frame.rows; ++i)
        {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += frame.step;
        }
    }

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
}

void CameraDriver::ImageCallback()
{
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    // fps기준으로 넘어가면 최신화
    if(!frame.empty() &&
       std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/fps_*1000)
    {
        last_frame_ = now;

        // Convert to a ROS2 image
        if(!is_flipped)
        {
            image_msg_ = ConvertFrameToMessage(frame);
        }
        else
        {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
            image_msg_ = ConvertFrameToMessage(frame);
        }

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        image_msg_->header.stamp = timestamp;
        image_msg_->header.frame_id = frame_id_;

        camera_info_msg_->header.stamp = timestamp;
        camera_info_msg_->header.frame_id = frame_id_;

        camera_info_pub_.publish(image_msg_, camera_info_msg_);
    }
}

