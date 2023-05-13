#include "usb_camera_driver.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto usb_camera_driver = std::make_shared<usb_camera_driver::CameraDriver>(options);

    exec.add_node(usb_camera_driver);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
