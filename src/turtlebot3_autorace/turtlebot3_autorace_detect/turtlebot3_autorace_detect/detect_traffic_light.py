#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Jun

from enum import Enum
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_sign')

        self.sub_image_type = 'raw'  # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                #'/detect/image_input/compressed',
                '/camera/image_compensated/compressed',
                self.cbFindTrafficSign,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                #'/detect/image_input',
                '/camera/image_compensated',
                self.cbFindTrafficSign,
                10
            )

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        # Publisher for detected traffic light color
        self.pub_traffic_sign = self.create_publisher(String, '/detect/traffic_light', 10)

        self.cvBridge = CvBridge()
        self.counter = 1
        self.fnPreproc()
        self.get_logger().info('DetectSign Node Initialized')

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        dir_path = get_package_share_directory('turtlebot3_autorace_detect')
        dir_path = os.path.join(dir_path, 'image')
        self.img_map = {
            'green': cv2.imread(os.path.join(dir_path, 'greenlight.png'), 0),
            'yellow': cv2.imread(os.path.join(dir_path, 'yellowlight.png'), 0),
            'red': cv2.imread(os.path.join(dir_path, 'redlight.png'), 0),
        }

        self.kp_map = {}
        self.des_map = {}

        for color, img in self.img_map.items():
            kp, des = self.sift.detectAndCompute(img, None)
            self.kp_map[color] = kp
            self.des_map[color] = des
        self.hsv_ranges = {
            'red': [
                # (np.array([0, 100, 100]), np.array([10, 255, 255])),
                # (np.array([160, 100, 100]), np.array([180, 255, 255]))
                (np.array([0, 150, 150]), np.array([10, 255, 255])),  # 더 엄격한 범위
                (np.array([170, 150, 150]), np.array([180, 255, 255]))  # 범위 조정
            ],
            'yellow': [
                (np.array([28, 150, 200]), np.array([33, 255, 255]))
            ],
            'green': [
                (np.array([58, 150, 150]), np.array([65, 255, 255]))
            ]
        }
        FLANN_INDEX_KDTREE = 0
        index_params = {
            'algorithm': FLANN_INDEX_KDTREE,
            'trees': 5
        }

        search_params = {
            'checks': 50
        }

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def detect_dominant_color(self, cv_img):
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        for color, ranges in self.hsv_ranges.items():
            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask_total += cv2.inRange(hsv, lower, upper)
            percentage = np.sum(mask_total) / (mask_total.shape[0] * mask_total.shape[1] * 255)
            if percentage > 0.003:  #0.001 -> 0.003
                return color
        return None

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1]
        err = total_sum / num_all
        return err

    def cbFindTrafficSign(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        img = cv_image_input.copy()
        h, w = img.shape[:2]
        
        # ROI 설정
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        roi_mask[0:140, 160:300] = 255
        y1, y2 = 0, 140
        x1, x2 = 160, 300

        masked_image = cv2.bitwise_and(img, img, mask=roi_mask)

        # Dominant color 검출
        dominant_color = self.detect_dominant_color(masked_image[y1:y2, x1:x2])
        
        # none 감지 시 처리
        if dominant_color is None:
            self.get_logger().info("No color detected - publishing 'none'")
            msg = String()
            msg.data = "none"
            self.pub_traffic_sign.publish(msg)
            
            result_img = masked_image
            
            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(result_img, 'jpg')
                )
            else:
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(result_img, 'bgr8')
                )
            return

        # SIFT 매칭 수행
        gray_roi = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        kp1, des1 = self.sift.detectAndCompute(gray_roi, None)
        
        if des1 is not None:
            des2 = self.des_map[dominant_color]
            kp2 = self.kp_map[dominant_color]
            matches = self.flann.knnMatch(des1, des2, k=2)

            good = [m for m, n in matches if m.distance < 0.7 * n.distance]
             #빨간 불일 경우에만 특징점 늘리기 
            if dominant_color == 'red':
                required_matches = 10
                mse_threshold = 50000
            else:
                required_matches = 6
                mse_threshold = 70000
            
            
            
            if len(good) >= required_matches:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

                mse = self.fnCalcMSE(src_pts, dst_pts)
                if mse < mse_threshold:
                    self.get_logger().info(f'{dominant_color.upper()} LIGHT DETECTED')
                    msg = String()
                    msg.data = dominant_color
                    self.pub_traffic_sign.publish(msg)

                    result_img = cv2.drawMatches(
                        masked_image, kp1, self.img_map[dominant_color], kp2, good,
                        None, matchColor=(255, 0, 0), flags=2
                    )
                else:
                    result_img = masked_image
            else:
                result_img = masked_image
        else:
            result_img = masked_image

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(result_img, 'jpg')
            )
        else:
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(result_img, 'bgr8')
            )

def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()