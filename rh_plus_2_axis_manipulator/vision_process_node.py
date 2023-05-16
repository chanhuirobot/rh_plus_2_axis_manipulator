#!/usr/bin/env python3

# Addison Sears-Collins https://automaticaddison.com 코드 참조
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from rclpy.qos import qos_profile_sensor_data # qos 세팅 가져오기
import numpy as np # numpy 가져오기

# 노드명 : VisionProcessor
# 기능 : 이미지 수신, 이미지로부터 좌표 뽑아내기, 좌표 publish
class VisionProcessor(Node):
    # 생성자
    def __init__(self):
        super().__init__('vision_processor')
        self.subscription = self.create_subscription(Image, 'image', self.listener_callback, qos_profile_sensor_data)
        self.br = CvBridge() # Used to convert between ROS and OpenCV images
        self.coord_x,self.coord_y = 0,0

    # 이미지 수신(+처리)
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        # 현재 이미지 넣고 좌표 최신화하기
        self.coord_x,self.coord_y = self.export_coordinate(current_frame)

    # 이미지로부터 좌표 뽑아내기
    def export_coordinate(self, frame):
        # 먼저 이미지 필요없는 부분들 잘라내기(화이트보드만 나오도록)
        # 좌상, 우상, 우하, 좌하 (시계 방향으로 4 지점 정의)
        src = np.array([[511, 352], [1008, 345], [1122, 584], [455, 594]], dtype=np.float32) # input 4개 지점
        dst = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32) # output 4개 지점
        matrix = cv2.getPerspectiveTransform(src, dst) # Matrix 얻어옴
        cropped = cv2.warpPerspective(frame, matrix, (width, height)) # matrix 대로 변환을 함

        img_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV) # cropped 이라는 이미지를 BGR에서 HSV색타입으로 변경

        # 어느정도 범위의 빨간색을 검출할 것인지 설정 (HSV 색타입)
        hue_red = 0
        lower_red = (hue_red-4, 160, 70)
        upper_red = (hue_red+4, 195, 115)
        img_mask = cv2.inRange(img_hsv, lower_red, upper_red)

        # 모폴로지 팽창을 3번 적용하여 노이즈(구멍) 메꾸기
        kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 5, 5 ) )
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
        if max_index != -1:
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

        # 중심 좌표 리턴
        return center_x, center_y

# main 함수
def main(args=None):
    rclpy.init(args=args)
    vision_processor = VisionProcessor()
    rclpy.spin(vision_processor)

    vision_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
