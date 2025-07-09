
#!/usr/bin/env python3

from enum import Enum
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Float64
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
class DetectPerson(Node):

    def __init__(self):
        super().__init__('detect_person')

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbFindPerson,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbFindPerson,
                10
            )

        self.pub_person_detected = self.create_publisher(UInt8, '/camera/person_detected', 10)
        if self.pub_image_type == 'compressed':
            self.pub_image_result = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_result = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.PersonEnum = Enum('PersonEnum', 'person')
        self.counter = 1

        self.fnPreproc()
        self.get_logger().info('DetectPerson Node Initialized')


        #움직이는 영역의 사람을 추출
       
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=False)

        #사람이 검출된다면 토픽을 발행
       
        # 사람 검출 알고리즘
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


        #차선정보 
        self.lane_center=500
        self.lane_state = 2  # 기본은 양쪽 차선 감지로 시작
        self.create_subscription(Float64, '/detect/lane', self.cbLaneCenter, 1)
        self.create_subscription(UInt8, '/detect/lane_state', self.cbLaneState, 1)

        self.diff_history = []  # 최근 signed_diff의 절대값 저장

        self.lane_x_bounds = [0, 319]  # 기본값, 왼쪽~오른쪽 전체
        self.create_subscription(Float64MultiArray, '/lane/x_bounds', self.cbLaneBounds, 1)


        self.person_detected = False


    def cbLaneBounds(self, msg):
        if len(msg.data) >= 2:
            # 보정 적용
            # self.get_logger().info(f'보정전 차선 범위 (픽셀 기준): {msg.data[0],msg.data[1]}')

            x_min = msg.data[0] / 1000.0 * 320.0
            x_max = msg.data[1] / 1000.0 * 320.0
            self.lane_x_bounds = [x_min, x_max]
            # self.get_logger().info(f'보정된 차선 범위 (픽셀 기준): {self.lane_x_bounds}')


    def cbLaneCenter(self, msg):
        self.lane_center = int(msg.data)

    def cbLaneState(self, msg):
        self.lane_state = msg.data

    def fnPreproc(self):
        self.sift = cv2.SIFT_create()

        dir_path = get_package_share_directory('turtlebot3_autorace_detect')
        dir_path = os.path.join(dir_path, 'image')

        # self.img_person = cv2.imread(dir_path + '/person_cropped.png', 0)  # trainImage2
        # self.kp_person, self.des_person = self.sift.detectAndCompute(self.img_person, None)

        FLANN_INDEX_KDTREE = 0
        index_params = {
            'algorithm': FLANN_INDEX_KDTREE,
            'trees': 5
        }
        search_params = {
            'checks': 50
        }
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1]
        return total_sum / num_all

    def cbFindPerson(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        # 이미지 디코딩
        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_purple = np.array([125, 100, 100])
        upper_purple = np.array([150, 255, 255])
        mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

        contours, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.person_detected = False  # 초기화

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 320:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            person_center = (x + w / 2)
            person_x_min = x
            person_x_max = x + w
            x_min = self.lane_x_bounds[0] - 60
            x_max = self.lane_x_bounds[1] + 60

            if person_x_max < x_min or person_x_min > x_max:
                self.get_logger().info(
                    f"무시: 박스 좌표({person_x_min:.1f}~{person_x_max:.1f})가 차선 범위({x_min:.1f}~{x_max:.1f}) 벗어남"
                )
                continue

            if self.lane_state == 0:
                self.get_logger().info("무시: 차선 미검출 상태")
                continue

            if 70 < person_center < 280:
                self.get_logger().info(f"감지됨: 중심 접근 (center={person_center:.1f})")
                self.person_detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                break
            elif person_center > 280:
                self.get_logger().info(f"무시: 너무 오른쪽에 있음 (center={person_center:.1f})")
            else:
                self.get_logger().info(f"무시: 너무 왼쪽에 있음 (center={person_center:.1f})")

        # ===== ✅ 여기서 한 번만 publish =====
        msg = UInt8()
        msg.data = 1 if self.person_detected else 0
        self.pub_person_detected.publish(msg)
        self.get_logger().info(f"사람 감지 결과: {msg.data}")

        # 시각화 이미지도 함께 publish
        if self.pub_image_type == 'compressed':
            self.pub_image_result.publish(self.cvBridge.cv2_to_compressed_imgmsg(frame, 'jpg'))
        else:
            self.pub_image_result.publish(self.cvBridge.cv2_to_imgmsg(frame, 'bgr8'))






def main(args=None):
    rclpy.init(args=args)
    node = DetectPerson()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
