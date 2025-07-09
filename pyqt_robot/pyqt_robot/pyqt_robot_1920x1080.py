import os
os.environ['OPENCV_QT_PLUGIN_PATH'] = ''
import PyQt5
import cv2

import pyqtgraph as pg
from std_msgs.msg import Float64  # /control/linear_vel 메시지 타입
from collections import deque
import time


import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QMainWindow,
    QPushButton, QLabel
)
from PyQt5.QtGui import QPixmap, QTransform, QPen, QImage
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor

from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
from PIL import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, UInt8


from ament_index_python.packages import get_package_share_directory
import os

package_name = 'pyqt_robot'
package_path = get_package_share_directory(package_name)

MAP_IMAGE_PATH = os.path.join(
    package_path,
    'resource/course.png'
)

RESIZED_MAP_PATH = "/tmp/resized_map.png"

ROBOT_IMAGE_PATH = os.path.join(
    package_path,
    'resource/robot.png'
)



def resize_map_image():
    """4m x 4m (520px x 520px)로 맵 이미지 리사이즈"""
    SCALE = 130
    size_px = int(4.0 * SCALE)

    img = Image.open(MAP_IMAGE_PATH)
    img = img.resize((size_px, size_px), Image.BILINEAR)
    img = img.rotate(180)
    img.save(RESIZED_MAP_PATH)
    print(f"Resized map saved to: {RESIZED_MAP_PATH}")

def quaternion_to_euler_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)

class OdomListener(Node, QObject):
    odom_signal = pyqtSignal(float, float, float)  # x, y, yaw
    image_signal = pyqtSignal(object)  # OpenCV image
    velocity_signal = pyqtSignal(float)
    control_state_signal = pyqtSignal(float, float,float)  # angular_z, error
    traffic_signal = pyqtSignal(str)
    person_signal = pyqtSignal(bool)
    

    def __init__(self):
        Node.__init__(self, 'odom_listener')
        QObject.__init__(self)
        self.bridge = CvBridge()
        # Odometry TF 구독
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            ROSImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Velocity
        self.vel_sub = self.create_subscription(
            Float64, 
            '/control/linear_vel', 
            self.velocity_callback, 
            10)
        # angular_z , error
        self.control_state_sub = self.create_subscription(
            Vector3, 
            '/debug/control_state', 
            self.control_state_callback, 
            10)

        # 신호등 상태 구독
        self.traffic_sub = self.create_subscription(
            String,
            '/detect/traffic_light',
            self.traffic_callback,
            10
        )

        # 사람 감지 구독
        self.person_sub = self.create_subscription(
            UInt8,
            '/camera/person_detected',
            self.person_callback,
            10
        )
    def traffic_callback(self, msg):
        self.traffic_signal.emit(msg.data)

    def person_callback(self, msg):
        detected = (msg.data == 1)
        self.person_signal.emit(detected)

    def control_state_callback(self, msg):
        # msg.x = angular_z, msg.y = error / 100.0, msg.z = pitch
        self.control_state_signal.emit(msg.x, msg.y, msg.z)


    def velocity_callback(self, msg):
        self.velocity_signal.emit(msg.data)


    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                trans = transform.transform.translation
                rot = transform.transform.rotation
                yaw = quaternion_to_euler_yaw(rot.x, rot.y, rot.z, rot.w)
                self.odom_signal.emit(trans.x, trans.y, yaw)
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

# ... [이전의 import 부분은 동일하게 유지] ...

class MapViewer(QGraphicsView):
    def __init__(self, odom_listener):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        
        # 뷰포트 설정 변경
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        
        scene_alpha = 0
        self.scene.setSceneRect(-1920-scene_alpha, -1080-scene_alpha, 3840+2*scene_alpha, 2160+2*scene_alpha)  # 1920x1080 기준으로 조정

        # Constants
        self.SCALE = 130.0  # pixels per meter
        self.MAP_ORIGIN_X = -2.0  # meters
        self.MAP_ORIGIN_Y = -2.0  # meters
        self.ROBOT_ICON_OFFSET = 16  # pixels
        
        # 고정점 설정 (1920x1080에 맞게 1500, -540으로 조정)
        self.fixed_point = QtCore.QPointF(1500, -540)

        # Load map image
        self.map_pixmap = QPixmap(RESIZED_MAP_PATH)
        self.map_item = QGraphicsPixmapItem(self.map_pixmap)
        self.scene.addItem(self.map_item)
        
        # Robot icon
        self.robot_item = QGraphicsPixmapItem(QPixmap(ROBOT_IMAGE_PATH))
        self.robot_item.setOffset(-self.ROBOT_ICON_OFFSET, -self.ROBOT_ICON_OFFSET)
        self.scene.addItem(self.robot_item)

        # 미니맵 원형 띠 크기 조정
        self.minimap_ring = QGraphicsEllipseItem(
            -300, -300, 600, 600  # 크기 조정
        )
        self.minimap_ring.setPen(QPen(Qt.yellow, 25))  # 두께 조정
        self.minimap_ring.setBrush(QBrush(Qt.NoBrush))
        self.minimap_ring.setZValue(5)
        self.minimap_ring.setPos(self.fixed_point)
        self.scene.addItem(self.minimap_ring)

        # 미니맵 원형 띠2 크기 조정
        self.minimap_ring2 = QGraphicsEllipseItem(
            -500, -500, 1000, 1000  # 크기 조정
        )
        self.minimap_ring2.setPen(QPen(QBrush(QColor(20, 20, 20)), 400))  # 두께 조정
        self.minimap_ring2.setBrush(QBrush(Qt.NoBrush))
        self.minimap_ring2.setZValue(4)
        self.minimap_ring2.setPos(self.fixed_point)
        self.scene.addItem(self.minimap_ring2)

        # UI setup
        self.setup_ui()

        # ROS connection
        odom_listener.odom_signal.connect(self.update_robot_position)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(50)

        self.odom_listener = odom_listener
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.i = 0

        # Center indicator
        self.center_dot = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.center_dot.setBrush(Qt.red)
        self.center_dot.setZValue(10)
        self.center_dot.setPos(self.fixed_point)
        self.scene.addItem(self.center_dot)

        # Initial setup
        self.mode = 'map_fixed'
        self.setBackgroundBrush(QBrush(QColor(20, 20, 20)))
        self.reset_view()

        # Camera view label (크기 조정)
        self.camera_label = QLabel(self)
        self.camera_label.setGeometry(20, 20, 480, 360)  # 크기 축소
        self.camera_label.setStyleSheet("border: 2px solid white;")
        self.camera_label.setScaledContents(True)

        # Connect image signal
        odom_listener.image_signal.connect(self.update_camera_view)

        # ----- Velocity Graph -----
        self.max_points = 300
        self.vel_data = deque(maxlen=self.max_points)
        self.time_data = deque(maxlen=self.max_points)
        self.start_time = time.time()

        self.plot_widget = pg.PlotWidget(self)
        self.plot_widget.setGeometry(530, 20, 600, 250)  # 위치 및 크기 조정
        self.plot_widget.setBackground('w')
        self.plot_widget.setTitle("Lin_x Vel", color='k', size='8pt')
        self.plot_widget.setLabel('left', 'Velocity (m/s)')
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_curve = self.plot_widget.plot(pen=pg.mkPen('b', width=2))

        odom_listener.velocity_signal.connect(self.update_velocity_plot)

        # ----- Angular Z & Error Graph -----
        self.max_points_debug = 300
        self.debug_time_data = deque(maxlen=self.max_points_debug)
        self.angular_z_data = deque(maxlen=self.max_points_debug)
        self.error_data = deque(maxlen=self.max_points_debug)
        self.pitch_data = deque(maxlen=self.max_points_debug)
        self.debug_start_time = time.time()

        self.debug_plot_widget = pg.PlotWidget(self)
        self.debug_plot_widget.setGeometry(530, 290, 600, 250)  # 위치 및 크기 조정
        self.debug_plot_widget.setBackground('w')
        self.debug_plot_widget.setTitle("Ang_Z Vel & Error(1/10 Scale)", color='k', size='8pt')
        self.debug_plot_widget.setLabel('left', 'Value')
        self.debug_plot_widget.setLabel('bottom', 'Time (s)')
        self.debug_plot_widget.showGrid(x=True, y=True)

        self.angular_curve = self.debug_plot_widget.plot(pen=pg.mkPen('r', width=2), name='Angular Z')
        self.error_curve = self.debug_plot_widget.plot(pen=pg.mkPen('g', width=2), name='Error')

        # pitch 그래프 (크기 조정)
        self.pitch_plot_widget = pg.PlotWidget(self)
        self.pitch_plot_widget.setGeometry(530, 560, 600, 200)  # 위치 및 크기 조정
        self.pitch_plot_widget.setBackground('w')
        self.pitch_plot_widget.setTitle("Pitch", color='k', size='8pt')
        self.pitch_plot_widget.setLabel('left', 'Pitch (degrees)')
        self.pitch_plot_widget.setLabel('bottom', 'Time (s)')
        self.pitch_plot_widget.showGrid(x=True, y=True)
        self.pitch_curve = self.pitch_plot_widget.plot(pen=pg.mkPen('b', width=2), name='Pitch')

        odom_listener.control_state_signal.connect(self.update_control_state)

        # 신호등 상태 메시지 라벨 (위치 조정)
        self.traffic_label = QLabel(self)
        self.traffic_label.setStyleSheet("color: white; font-size: 24px;")  # 폰트 크기 축소
        self.traffic_label.move(20, 520)  # 위치 조정
        self.traffic_label.resize(600, 40)  # 크기 조정

        # 사람 감지 메시지 라벨 (위치 조정)
        self.person_label = QLabel(self)
        self.person_label.setStyleSheet("color: white; font-size: 24px;")  # 폰트 크기 축소
        self.person_label.move(20, 560)  # 위치 조정
        self.person_label.resize(600, 40)  # 크기 조정

        # 시그널 연결
        odom_listener.traffic_signal.connect(self.update_traffic_message)
        odom_listener.person_signal.connect(self.update_person_message)

    def setup_ui(self):
        # 모드 전환 버튼 위치 조정
        self.toggle_button = QPushButton("Switch to Robot-Fixed Mode", self)
        self.toggle_button.move(1320, 20)  # 위치 조정
        self.toggle_button.resize(250, 30)  # 크기 조정
        self.toggle_button.clicked.connect(self.toggle_mode)


    def update_traffic_message(self, state):
        if state == "red":
            self.traffic_label.setText("빨간불이 감지되었습니다. 정지합니다.")
        elif state == "yellow":
            self.traffic_label.setText("노란불이 감지되었습니다. 서행합니다.")
        elif state == "green":
            self.traffic_label.setText("초록불이 감지되었습니다. 전진합니다.")
        else:
            self.traffic_label.setText("신호등 감지되지 않음")

    def update_person_message(self, detected):
        if detected:
            self.person_label.setText("사람이 감지되었습니다")
        else:
            self.person_label.setText("")


    def update_control_state(self, angular_z, error, pitch):
        t = time.time() - self.debug_start_time
        self.debug_time_data.append(t)
        self.angular_z_data.append(angular_z)
        self.error_data.append(error)
        self.pitch_data.append(pitch)

        self._update_debug_plot()
        self._update_pitch_plot()

    def _update_debug_plot(self):
        self.angular_curve.setData(self.debug_time_data, self.angular_z_data)
        self.error_curve.setData(self.debug_time_data, self.error_data)

    def _update_pitch_plot(self):
        self.pitch_curve.setData(self.debug_time_data, self.pitch_data)


    def update_velocity_plot(self, vel):
        t = time.time() - self.start_time
        self.vel_data.append(vel)
        self.time_data.append(t)
        self.plot_curve.setData(self.time_data, self.vel_data)

    def update_camera_view(self, cv_image):
        # OpenCV BGR -> RGB 변환
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # OpenCV 이미지 → QImage → QPixmap
        height, width, channel = rgb_image.shape
        bytes_per_line = 3 * width
        qimage = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        # QLabel에 표시
        self.camera_label.setPixmap(pixmap)



    def setup_ui(self):
        # 모드 전환 버튼을 오른쪽 위로 이동
        self.toggle_button = QPushButton("Switch to Robot-Fixed Mode", self)
        self.toggle_button.move(self.width() - 80, 280)  # 오른쪽 위에 배치
        self.toggle_button.resize(400, 50)
        self.toggle_button.clicked.connect(self.toggle_mode)

        # 고정점 좌표 표시 제거
        # 기존의 label_x, label_y, label_mode, label_fixed_point 제거

    def reset_view(self):
        """Reset view to initial state based on current mode"""
        if self.mode == 'map_fixed':
            # Center the map at fixed point
            self.map_item.setPos(0, 0)
            self.map_item.setTransform(QTransform())
            
            # Position map so its center is at fixed point
            map_center_x = self.fixed_point.x() - self.map_pixmap.width()/2
            map_center_y = self.fixed_point.y() - self.map_pixmap.height()/2
            self.map_item.setPos(map_center_x, map_center_y)
            
            # Update center dot
            self.center_dot.setPos(self.fixed_point)
            self.scene_center = self.fixed_point
        else:
            # Center the robot at fixed point
            self.robot_item.setPos(self.fixed_point)
            self.robot_item.setTransform(QTransform().rotate(-90))  # Point robot north
            
            # Update center dot
            self.center_dot.setPos(self.fixed_point)
            self.scene_center = self.fixed_point
        
        # 미니맵 띠 위치 업데이트
        self.minimap_ring.setPos(self.fixed_point)
        self.minimap_ring2.setPos(self.fixed_point)        

        # Ensure the fixed point is visible
        self.centerOn(self.fixed_point)
        self.ensureVisible(self.fixed_point.x(), self.fixed_point.y(), 100, 100)

    def toggle_mode(self):
        if self.mode == 'map_fixed':
            self.mode = 'robot_fixed'
            self.toggle_button.setText("Switch to Map-Fixed Mode")
        else:
            self.mode = 'map_fixed'
            self.toggle_button.setText("Switch to Robot-Fixed Mode")
            
        self.reset_view()
        self.update_robot_position(self.current_x, self.current_y, self.current_theta)

    def spin_once(self):
        rclpy.spin_once(self.odom_listener, timeout_sec=0.01)

    def update_robot_position(self, x, y, theta):
        self.current_x = x
        self.current_y = y
        self.current_theta = theta

        # Convert real-world coordinates to pixels
        px = (x - self.MAP_ORIGIN_X) * self.SCALE
        py = (y - self.MAP_ORIGIN_Y) * self.SCALE
        map_height_px = self.map_pixmap.height()

        if self.mode == 'map_fixed':
            # Map is fixed at fixed point, move robot
            robot_x = px
            robot_y = map_height_px - py
            
            # Position robot relative to map
            robot_scene_x = self.map_item.pos().x() + robot_x
            robot_scene_y = self.map_item.pos().y() + robot_y
            
            self.robot_item.setPos(robot_scene_x, robot_scene_y)
            self.robot_item.setTransform(QTransform().rotate(-theta))
            
            # Keep fixed point visible
            self.centerOn(self.fixed_point)
        else:
            # Robot is fixed at fixed point, move map
            # Calculate map position to keep robot at fixed point
            map_x = self.fixed_point.x() - px
            map_y = self.fixed_point.y() - (map_height_px - py)
            
            # Apply transformation
            transform = QTransform()
            transform.translate(self.fixed_point.x(), self.fixed_point.y())
            transform.rotate(theta - 90)  # Adjust for robot orientation
            transform.translate(-px, -(map_height_px - py))
            
            self.map_item.setTransform(transform)
            self.map_item.setPos(0, 0)  # Reset position before transform
            
            # Keep robot at fixed point
            self.robot_item.setPos(self.fixed_point)
            self.centerOn(self.fixed_point)

        if self.i == 0:
            self.debug_positions()
            self.i = 30
        self.i -= 1

    def debug_positions(self):
        print(f"\nMode: {self.mode}")
        print(f"Robot scene pos: ({self.robot_item.pos().x():.1f}, {self.robot_item.pos().y():.1f})")
        print(f"Map scene pos: ({self.map_item.pos().x():.1f}, {self.map_item.pos().y():.1f})")
        print(f"Fixed point: ({self.fixed_point.x():.1f}, {self.fixed_point.y():.1f})")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.toggle_button.move(self.width() - 650, 20)
        self.traffic_label.move(140, self.height() - 360)
        self.person_label.move(140, self.height() - 300)
        self.reset_view()
        self.update_robot_position(self.current_x, self.current_y, self.current_theta)


# class MainWindow(QMainWindow):
#     def __init__(self, odom_listener):
#         super().__init__()
#         self.setWindowTitle("2D Map Viewer")
#         self.setCentralWidget(MapViewer(odom_listener))
#         self.resize(2560, 1440)
class MainWindow(QMainWindow):
    def __init__(self, odom_listener):
        super().__init__()
        self.setWindowTitle("2D Map Viewer")
        self.setCentralWidget(MapViewer(odom_listener))
        self.resize(1920, 1080)  # 해상도 변경
        #self.resize(1706, 960)  # 해상도 변경
# ... [나머지 코드는 동일하게 유지] ...

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'
def main():
    resize_map_image()
    rclpy.init()
    odom_listener = OdomListener()
    app = QApplication(sys.argv)
    window = MainWindow(odom_listener)
    window.show()
    sys.exit(app.exec_())
    odom_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

