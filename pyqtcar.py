from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QMainWindow
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtCore import QTimer
import sys
import math

class MapViewer(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # 맵 이미지 (ex. occupancy grid로 생성한 지도)
        self.map_item = QGraphicsPixmapItem(QPixmap("/home/bako98/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace_2020/course/materials/textures/course.png"))  # 지도 이미지
        self.scene.addItem(self.map_item)

        # 로봇 아이콘 (위에서 본 모습의 로봇 이미지)
        self.robot_item = QGraphicsPixmapItem(QPixmap("/home/bako98/turtlebot3_ws/src/robot.png"))
        self.robot_item.setOffset(-16, -16)  # 중심 정렬
        self.scene.addItem(self.robot_item)

        # 로봇 위치 업데이트용 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start(100)  # 10Hz 업데이트

        self.x = 100
        self.y = 100
        self.theta = 0

    def update_robot_position(self):
        # 여기에 ROS에서 받은 위치 데이터를 넣으면 됩니다
        self.x += 1
        self.theta += 5

        self.robot_item.setPos(self.x, self.y)

        transform = QTransform()
        transform.rotate(self.theta)
        self.robot_item.setTransform(transform)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2D Map Viewer")
        self.setCentralWidget(MapViewer())

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
