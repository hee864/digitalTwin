import numpy as np
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String, UInt8
from nav_msgs.msg import Odometry
import math
from enum import Enum
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def euler_from_quaternion(msg):
    x, y, z, w = msg.x, msg.y, msg.z, msg.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class RobotState(Enum):
    NORMAL = 0
    PAUSED_BY_PERSON = 1
    PAUSED_BY_RED_LIGHT = 2
    LOW_BY_YELLOW_LIGHT = 3

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        # Subscribers
        self.create_subscription(UInt8, '/detect/lane_state', self.callback_get_lane_state, 1)
        self.create_subscription(Float64, '/control/lane', self.callback_get_center, 1)
        self.create_subscription(Float64, '/detect/white_center', self.callback_get_white_center, 1)
        self.create_subscription(String, '/detect/traffic_sign', self.callback_get_traffic_sign, 1)
        self.create_subscription(String, '/detect/traffic_light', self.callback_get_traffic_light, 1)
        self.create_subscription(String, '/traffic_light_state', self.traffic_light_callback, 1)
        self.create_subscription(UInt8, '/camera/person_detected', self.person_detected_callback, 1)
        self.create_subscription(Float64, '/control/max_vel', self.callback_get_max_vel, 1)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu,'/imu',self.callback_imu,1)

        self.pub_linear_vel = self.create_publisher(Float64, '/control/linear_vel', 1)
        self.pub_debug_vector = self.create_publisher(Vector3, '/debug/control_state', 1)

        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        # State variables
        self.filtered_pitch = 0.0
        self.pitch = 0.0
        self.vel_offset = 0.0
        self.state = RobotState.NORMAL
        self.person_detected = False
        self.pause_end_time = 0
        self.traffic_light_state = "none"
        self.traffic_sign_state = ""
        self.intersection_state = False
        self.escape_tracking = False
        self.heading_when_escape_started = None
        self.yaw = 0.0
        self.turning = False
        self.turning_start_time = None
        self.lane_center = 0.0
        self.white_lane_center = 0.0
        self.lane_state = 0
        self.last_error = 0.0
        self.MAX_VEL = 0.1

        #추가
        self.red_last_seen = None
        self.yellow_last_seen = None

        self.imu_ready = False
        self.imu_count = 0

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
       # 매번 초기화
        self.red_last_seen = None
        self.yellow_last_seen = None
        #빨간, 노란불 직전 감지 이후 2초 이내일 경우에만 상태 유지 → 끊기면 곧바로 해제됨
        # 빨간불 감지
        if self.traffic_light_state == "red":
            self.red_last_seen = now_sec

        # 노란불 감지
        elif self.traffic_light_state == "yellow":
            self.yellow_last_seen = now_sec

        # === 상태 결정 ===
        if self.person_detected and now_sec < self.pause_end_time:
            self.state = RobotState.PAUSED_BY_PERSON

        elif self.red_last_seen and now_sec - self.red_last_seen <= 2:
            self.state = RobotState.PAUSED_BY_RED_LIGHT

        elif self.yellow_last_seen and now_sec - self.yellow_last_seen <= 2:
            self.state = RobotState.LOW_BY_YELLOW_LIGHT

        else:
            self.state = RobotState.NORMAL


        # if self.traffic_light_state == "red":
        #     self.state = RobotState.PAUSED_BY_RED_LIGHT
        # elif self.person_detected and now_sec < self.pause_end_time:
        #     self.state = RobotState.PAUSED_BY_PERSON
        # elif self.traffic_light_state == "yellow":
        #     self.state = RobotState.LOW_BY_YELLOW_LIGHT
        # else:
        #     self.state = RobotState.NORMAL

        # 상태에 따른 동작
        if self.state == RobotState.PAUSED_BY_RED_LIGHT:
            self.get_logger().info(" 빨간불 - 정지 중")
            self.follow_lane(stop=True)
            return

        elif self.state == RobotState.PAUSED_BY_PERSON:
            self.get_logger().info(" 사람 감지 - 정지 중")
            self.follow_lane(stop=True)
            return

        elif self.state == RobotState.LOW_BY_YELLOW_LIGHT:
            self.get_logger().info("노란불 - 느리게 주행")
            self.follow_lane(slow=True)
            return
        
        #--------------------고침
        elif self.traffic_light_state == "green":
        #-------------------
            self.get_logger().info(" 초록불 - 일반 주행")

        if self.turning:
            elapsed = (self.get_clock().now() - self.turning_start_time).nanoseconds / 1e9
            if elapsed > 1.0:
                self.pub_cmd_vel.publish(Twist())
                self.turning = False
                self.get_logger().info("회전 종료")
            else:
                twist = Twist()
                twist.angular.z = -1.0
                self.pub_cmd_vel.publish(twist)
            return

        if self.escape_tracking:
            yaw_now_deg = np.degrees(self.yaw) % 360
            yaw_start_deg = np.degrees(self.heading_when_escape_started) % 360
            angle_diff = self.angle_diff_deg(yaw_now_deg, yaw_start_deg)

            if angle_diff > 65 : 
                self.get_logger().info(f'angle_diff={angle_diff}')
                self.get_logger().info("탈출 완료 → 일반 주행 복귀")
                self.escape_tracking = False
                self.heading_when_escape_started = None
                self.intersection_state = False
            else:
                # self.get_logger().info("탈출 중: 흰색선 추적")
                self.get_logger().info(
                    f"탈출 중:  angle_diff={angle_diff:.1f}"
                )

                self.follow_white_lane()
            return

        if self.traffic_sign_state == "intersection":
            self.get_logger().info("교차로 진입 감지")
            self.intersection_state = True
            self.traffic_sign_state = ""

        if self.intersection_state and self.traffic_sign_state == "right" and self.lane_state == 0:
            self.get_logger().info("우회전 조건 충족 → 회전 시작")
            self.pub_cmd_vel.publish(Twist())
            self.turning = True
            self.turning_start_time = self.get_clock().now()
            self.traffic_sign_state = ""
            return

        if self.intersection_state and self.traffic_sign_state == "intersection_escape":
            self.get_logger().info("교차로 탈출 감지 → 흰색 라인 추적")
            self.heading_when_escape_started = self.yaw
            self.escape_tracking = True
            self.intersection_state = False
            self.traffic_sign_state = ""
            return

        self.follow_lane()

    def follow_lane(self, slow=False, stop=False):
        twist = Twist()
        if stop:
            twist.linear.x = 0.0
            self.pub_cmd_vel.publish(twist)
            self.pub_linear_vel.publish(Float64(data=twist.linear.x))
            return

        error = self.lane_center - 160
        #이미지의 가로 중심이 500 , 차선의 중심이 오늘쪽에있으면 양수 왼쪽에 있으면 음수
        Kp = 0.004  
        Kd = 0.015
        angular_z = Kp * error + Kd * (error - self.last_error)
        #에러가 클수록 급하게 회전, 이전 eroor와 비교하여 급겹한 회전에도 반응
        
        self.last_error = error


        # IMU 보정 포함 속도 계산
        target_vel = self.MAX_VEL + self.vel_offset
        target_vel = max(min(target_vel, 0.2), 0.02)  # 안전한 범위 제한

        # twist.linear.x = min(target_vel * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        linear_x = max(min(target_vel * (max(1 - abs(error) / 160, 0) ** 2.2), 0.1),0)
        if slow:
            twist.linear.x = linear_x / 2
        else: #정속 주행
            twist.linear.x = linear_x 
        twist.angular.z = (-max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0))
        self.pub_cmd_vel.publish(twist)
        self.pub_linear_vel.publish(Float64(data=twist.linear.x))
        
        ##파이 큐디용
        msg = Vector3()
        msg.x = angular_z
        msg.y = error / 100.0
        msg.z = self.filtered_pitch
        self.pub_debug_vector.publish(msg)

    def follow_white_lane(self):
        error = self.white_lane_center - 160
        angular_z = 0.0025 * error + 0.007 * (error - self.last_error)
        self.last_error = error
        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 160, 0) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

         # ✅ PyQt 디버깅 정보 발행
        msg = Vector3()
        msg.x = angular_z
        msg.y = error / 100.0
        msg.z = self.filtered_pitch
        self.pub_debug_vector.publish(msg)

    def callback_imu(self, imu_msg):
        alpha = 0.1  # 필터 강도 (0.0: 무반응, 1.0: 반응 빠름)
        # pitch 계산
        q = imu_msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.asin(sinp)

        # 초기 10프레임은 필터 안정화만 수행 추가
        self.imu_count += 1
        if self.imu_count < 10:
            self.filtered_pitch = self.pitch  # 초기엔 그냥 복사
            self.vel_offset = 0.0
            return
        else:
            self.imu_ready = True

        # pitch 변화 감지용 비교
        prev_filtered_pitch = self.filtered_pitch
        prev_vel_offset = self.vel_offset
        
        # 저역통과 필터 적용
        self.filtered_pitch = (1 - alpha) * self.filtered_pitch + alpha * self.pitch
        # pitch가 +면 오르막 → 속도 증가, 내리막 → 속도 감소
        self.vel_offset = max(min(-self.filtered_pitch * 0.2, 0.1), -0.1)-0.05

        # 변화량 기준
        delta_pitch = abs(self.filtered_pitch - prev_filtered_pitch)
        delta_vel = abs(self.vel_offset - prev_vel_offset)

        # 기준 이상일 경우에만 로그 출력
        if delta_pitch > 0.01 or delta_vel > 0.005:
            self.get_logger().info(
                f"[IMU] pitch: {math.degrees(self.pitch):.2f}°, filtered: {math.degrees(self.filtered_pitch):.2f}°, vel_offset: {self.vel_offset:.4f}"
            )

    def angle_diff_deg(self, a_deg, b_deg):
        return abs((a_deg - b_deg + 180) % 360 - 180)

    def odom_callback(self, msg):
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation)[2]

    def callback_get_lane_state(self, msg):
        self.lane_state = msg.data

    def callback_get_traffic_light(self, msg):
        self.traffic_light_state = msg.data

    def callback_get_traffic_sign(self, msg):
        self.traffic_sign_state = msg.data

    def callback_get_center(self, msg):
        self.lane_center = msg.data

    def callback_get_white_center(self, msg):
        self.white_lane_center = msg.data

    def callback_get_max_vel(self, msg):
        self.MAX_VEL = msg.data

    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data

    def person_detected_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("사람 감지됨: 2초간 정지")
            self.person_detected = True
            self.pause_end_time = self.get_clock().now().seconds_nanoseconds()[0] + 2
        else:
            self.person_detected = False

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    rclpy.get_default_context().on_shutdown(node.shut_down)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
