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
from PyQt5.QtGui import QPixmap, QTransform, QPen
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor

from PIL import Image

MAP_IMAGE_PATH = "/home/bako98/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace_2020/course/materials/textures/course.png"
RESIZED_MAP_PATH = "/tmp/resized_map.png"

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

    def __init__(self):
        Node.__init__(self, 'odom_listener')
        QObject.__init__(self)
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                trans = transform.transform.translation
                rot = transform.transform.rotation
                yaw = quaternion_to_euler_yaw(rot.x, rot.y, rot.z, rot.w)
                self.odom_signal.emit(trans.x, trans.y, yaw)

class MapViewer(QGraphicsView):
    def __init__(self, odom_listener):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        
        # 뷰포트 설정 변경
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        
        scene_alpha = 0
        self.scene.setSceneRect(-2560-scene_alpha, -1440-scene_alpha, 5120+2*scene_alpha, 2880+2*scene_alpha)  # 씬 범위 확장

        # Constants
        self.SCALE = 130.0  # pixels per meter
        self.MAP_ORIGIN_X = -2.0  # meters
        self.MAP_ORIGIN_Y = -2.0  # meters
        self.ROBOT_ICON_OFFSET = 16  # pixels
        
        # 고정점 설정 (2000, -720)
        self.fixed_point = QtCore.QPointF(2000, -720)

        # Load map image
        self.map_pixmap = QPixmap(RESIZED_MAP_PATH)
        self.map_item = QGraphicsPixmapItem(self.map_pixmap)
        self.scene.addItem(self.map_item)
        
        # Robot icon
        self.robot_item = QGraphicsPixmapItem(QPixmap("/home/bako98/turtlebot3_ws/src/robot.png"))
        self.robot_item.setOffset(-self.ROBOT_ICON_OFFSET, -self.ROBOT_ICON_OFFSET)
        self.scene.addItem(self.robot_item)

        # 미니맵 원형 띠 추가
        self.minimap_ring = QGraphicsEllipseItem(
            -380, -380, 760, 760  # 반지름 300 + 두께 30 = 330
        )
        self.minimap_ring.setPen(QPen(Qt.yellow, 30))  # 노란색, 두께 30
        self.minimap_ring.setBrush(QBrush(Qt.NoBrush))  # QBrush로 감싼 NoBrush
        self.minimap_ring.setZValue(5)  # 로봇 아이콘보다 아래에 표시
        self.minimap_ring.setPos(self.fixed_point)
        self.scene.addItem(self.minimap_ring)

        # 미니맵 원형 띠2 추가
        self.minimap_ring2 = QGraphicsEllipseItem(
            -500, -500, 1000, 1000  # 반지름 500
        )
        self.minimap_ring2.setPen(QPen(QBrush(QColor(20, 20, 20)), 280))  # 회색, 두께 280
        self.minimap_ring2.setBrush(QBrush(Qt.NoBrush))  # QBrush로 감싼 NoBrush
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
        # 창 크기 변경 시 버튼 위치 조정
        self.toggle_button.move(self.width() - 800, 220)
        self.reset_view()
        self.update_robot_position(self.current_x, self.current_y, self.current_theta)

class MainWindow(QMainWindow):
    def __init__(self, odom_listener):
        super().__init__()
        self.setWindowTitle("2D Map Viewer")
        self.setCentralWidget(MapViewer(odom_listener))
        self.resize(2560, 1440)

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