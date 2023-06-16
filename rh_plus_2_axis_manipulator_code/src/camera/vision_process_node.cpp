#include "camera/vision_process_node.hpp"

// 생성자 선언. 노드 이름 : vision_process_node
VisionProcessNode::VisionProcessNode() : Node("vision_process_node")
{
  // 멤버 변수 초기화 (시작 위치)
  coord_x_ratio_ = 0.5;
  coord_y_ratio_ = 0.5;

  // 10초마다 moveit2_controller 발동시키기
  this->moveit_timer_ = this->create_wall_timer(std::chrono::milliseconds(10000), std::bind(&VisionProcessNode::moveit2_controller, this));

  // QoS 설정, 스브스크라이버 설정
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image", QOS_RKL10V, std::bind(&VisionProcessNode::image_process_callback, this, std::placeholders::_1));
}

// 이미지 수신, 좌표 추적(멤버 변수로 있는 좌표 최신화)
void VisionProcessNode::image_process_callback(sensor_msgs::msg::Image::SharedPtr data)
{
  cv::Mat img = cv_bridge::toCvShare(data, "bgr8")->image;

  // 먼저 이미지 필요없는 부분들 잘라내기(화이트보드만 나오도록)
  int output_width = 600;
  int output_height = 600;
  // Warping 전의 이미지 상의 좌표. 좌상, 우상, 우하, 좌하 (시계 방향으로 4 지점 정의)
  cv::Point2f corners[4] = {cv::Point2f(349, 65), cv::Point2f(887, 67), cv::Point2f(889, 682), cv::Point2f(347, 682)};
  // Warping 후의 좌표
  cv::Point2f warpCorners[4] = {cv::Point2f(0, 0), cv::Point2f(output_width, 0), cv::Point2f(output_width, output_height), cv::Point2f(0, output_height)};
  cv::Size warpSize(output_width, output_height);
  cv::Mat img_warp(warpSize, img.type());
  // Transformation Matrix 구하기
  cv::Mat trans = getPerspectiveTransform(corners, warpCorners);
  // Warping
  warpPerspective(img, img_warp, trans, warpSize);
  // 변환 완료

  // 이제 빨간색만 따기 위해 BGR 타입으로 선언된 이미지를 HSV로 바꾼다.
  cv::Mat img_hsv;
  cv::cvtColor(img_warp, img_hsv, cv::COLOR_BGR2HSV);
  // 이제 빨간색만 추출해서 img_red에 저장
  int hue_red = 0;
  cv::Scalar lower_red = cv::Scalar(hue_red-4, 150, 60);
	cv::Scalar upper_red = cv::Scalar(hue_red+10, 255, 150);
  cv::Mat img_red;
	inRange(img_hsv, lower_red, upper_red, img_red);

  // 모폴로지 팽창을 3번 적용하여 노이즈(구멍) 메꾸기
  // 결과는 img_end에 저장
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4,4));
  cv::Mat img_end;
  cv::morphologyEx(img_red, img_end, cv::MORPH_DILATE, kernel, cv::Point(-1,-1), 3);

  // labeling 진행. img_end 흰색 영역을 별도의 영역으로 분리.
  cv::Mat labels, stats, centroids;
  int nlabels = cv::connectedComponentsWithStats(img_end, labels, stats, centroids);

  // 가장 큰 영역의 빨간색 덩어리만 사용하므로
  double max = -1;
  int max_index = -1;
  for(int i = 0; i < nlabels; i++){
    if(i < 1) continue;
    double area = stats.at<double>(i, cv::CC_STAT_AREA);
    if(area > max){
      max = area;
      max_index = i;
    }
  }

  // 가장 큰 영역에서 중심 좌표, 너비, 높이 등을 파악
  int center_x = int(centroids.at<double>(max_index, 0));
  int center_y = int(centroids.at<double>(max_index, 1));
  int left = stats.at<int>(max_index, cv::CC_STAT_LEFT);
  int top = stats.at<int>(max_index, cv::CC_STAT_TOP);
  int width = stats.at<int>(max_index, cv::CC_STAT_WIDTH);
  int height = stats.at<int>(max_index, cv::CC_STAT_HEIGHT);

  // 빨간색 사각형과 초록색 점 그리기
  cv::rectangle(img_warp, cv::Rect(left, top, width, height), cv::Scalar(0,0,255), 5);
  cv::circle(img_warp, cv::Point(center_x, center_y), 10, cv::Scalar(0,255,0), -1);

  // 화면 출력
  cv::imshow("original", img);
  cv::imshow("cropped", img_warp);
  cv::imshow("mask", img_end);
  cv::waitKey(1);

  // 원했던 목표 : 좌표 비율 구하고 적용해주기 -> 최신화 완료
  coord_x_ratio_ = (double)center_x / (double)output_width;
  coord_y_ratio_ = (double)(output_height - center_y) / (double)output_height;
}

// 10초를 세고 moveit을 plan 하고 execute 한다.
// 안되면 안된다고 에러 띄우기(ex 닿을 수 있는 거리 밖이면)
void VisionProcessNode::moveit2_controller(){
  // countdown 구현
  for(int countdown_num = 10; countdown_num > 0; countdown_num--){
    RCLCPP_INFO(logger, "Countdown for next Moving: %ds / current coord ratio : (%.3f,%.3f)", countdown_num, coord_x_ratio_, coord_y_ratio_);
    // delay(1000);
  }


  // 이제 moveit 제어 시작
  // Target Pose 객체 생성
  geometry_msgs::msg::Pose target_pose;
  // set target pose
  target_pose.orientation.w = 1.0;
  target_pose.orientation.z = 0.0;
  // ratio(0.0~1.0) 에다가 길이를 곱해서 target.position 생성
  target_pose.position.x = coord_x_ratio_ * BOARD_WIDTH;
  target_pose.position.y = coord_y_ratio_ * BOARD_HEIGHT;

  // moveit의 plan 생성
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // Plan 하고 Execute 하기
  move_group_interface.setPoseTarget(target_pose);
  auto success = static_cast<bool>(move_group_interface.plan(plan));
  if(success){
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Success! Coord x = %.3f / y = %.3f", target_pose.position.x, target_pose.position.y);
  }
  else RCLCPP_ERROR(logger, "Planning failed! Maybe Out of Range...! :(");
}

// main 함수
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionProcessNode>());
  rclcpp::shutdown();
  return 0;
}
