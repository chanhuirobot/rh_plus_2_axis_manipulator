#!/usr/bin/env python3

# Addison Sears-Collins https://automaticaddison.com 코드 참조
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from rclpy.qos import qos_profile_sensor_data # qos 세팅 가져오기
from rclpy.qos import qos_profile_system_default
import numpy as np # numpy 가져오기
from rh_plus_interface.msg import CoordRatio

# 노드명 : VisionProcessor
# 기능 : 이미지 수신, 이미지로부터 좌표 뽑아내기, 좌표 publish
class VisionProcessor(Node):
    # 생성자
    def __init__(self):
        super().__init__('vision_processor')

        # 토픽 서브스크라이브 + 좌표 처리
        self.subscription = self.create_subscription(Image, 'image', self.listener_callback, qos_profile_sensor_data)

        self.br = CvBridge() # Used to convert between ROS and OpenCV images
        self.coord_x_ratio,self.coord_y_ratio = 0.5,0.5

        # 토픽 퍼블리시
        self.publication = self.create_publisher(CoordRatio, 'xy_coordinate_ratio', qos_profile_system_default)
        self.timer = self.create_timer(0.5, self.publish_coordinate_ratio)

    # 이미지 수신(+처리)
    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        # 현재 이미지 넣고 좌표 최신화하기
        self.coord_x_ratio,self.coord_y_ratio = self.export_coordinate(current_frame)
        # 좌표 출력
        self.get_logger().info("center_x_ratio : {0:0.3f} / center_y_ratio : {1:0.3f}".format(self.coord_x_ratio, self.coord_y_ratio))

    # 이미지로부터 좌표 뽑아내기
    def export_coordinate(self, frame):
        # 먼저 이미지 필요없는 부분들 잘라내기(화이트보드만 나오도록)
        # 좌상, 우상, 우하, 좌하 (시계 방향으로 4 지점 정의)
        output_width = 600
        output_height = 600
        src = np.array([[349, 65], [887, 67], [889, 682], [347, 682]], dtype=np.float32) # input 4개 지점
        dst = np.array([[0, 0], [output_width, 0], [output_width, output_height], [0, output_height]], dtype=np.float32) # output 4개 지점
        matrix = cv2.getPerspectiveTransform(src, dst) # Matrix 얻어옴
        cropped = cv2.warpPerspective(frame, matrix, (output_width, output_height)) # matrix 대로 변환을 함

        img_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV) # cropped 이라는 이미지를 BGR에서 HSV색타입으로 변경

        # 어느정도 범위의 빨간색을 검출할 것인지 설정 (HSV 색타입)
        hue_red = 0
        lower_red = (hue_red-4, 150, 60)
        upper_red = (hue_red+10, 255, 150)
        img_mask = cv2.inRange(img_hsv, lower_red, upper_red)

        # 모폴로지 팽창을 3번 적용하여 노이즈(구멍) 메꾸기
        kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 4, 4 ) )
        img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_DILATE, kernel, iterations = 3)

        # labeling 진행. img_mask 흰색 영역을 별도의 영역으로 분리.
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_mask)

        # 가장 큰 영역 사용
        max = -1
        max_index = -1
        for i in range(nlabels):
            if i < 1:
                continue
            area = stats[i, cv2.CC_STAT_AREA]
            if area > max:
                max = area
                max_index = i

        # 가장 큰 영역에서 중심 좌표, 너비, 높이 등을 파악
        center_x = int(centroids[max_index, 0])
        center_y = int(centroids[max_index, 1])
        left = stats[max_index, cv2.CC_STAT_LEFT]
        top = stats[max_index, cv2.CC_STAT_TOP]
        width = stats[max_index, cv2.CC_STAT_WIDTH]
        height = stats[max_index, cv2.CC_STAT_HEIGHT]

        # 빨간색 사각형과 초록색 점 그리기
        cv2.rectangle(cropped, (left, top), (left + width, top + height), (0, 0, 255), 5)
        cv2.circle(cropped, (center_x, center_y), 10, (0, 255, 0), -1)

        # 화면 출력
        cv2.imshow("original", frame)
        cv2.imshow("cropped", cropped)
        cv2.imshow('mask', img_mask)
        cv2.waitKey(1)

        # 중심 좌표 비율 계산
        center_x_ratio = float(center_x / output_width)
        center_y_ratio = float((output_height - center_y) / output_height)

        # 중심 좌표 비율 리턴
        return center_x_ratio, center_y_ratio

    # 좌표 publish 함수
    def publish_coordinate_ratio(self):
        msg = CoordRatio()
        msg.coord_x_ratio = self.coord_x_ratio
        msg.coord_y_ratio = self.coord_y_ratio
        self.publication.publish(msg)

# main 함수
def main(args=None):
    rclpy.init(args=args)
    vision_processor = VisionProcessor()
    rclpy.spin(vision_processor)

    vision_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
