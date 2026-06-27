# Architecture and Detailed Design

## 컴포넌트

| ID | 컴포넌트 | 책임 | 구현 위치 |
| --- | --- | --- | --- |
| DT-CMP-001 | Lane Detector | 차선 전처리, contour, 중심 계산 | `turtlebot3_autorace_detect/detect_lane.py` |
| DT-CMP-002 | Traffic Detector | SIFT 기반 신호·표지 인식 | `turtlebot3_autorace_detect/detect_traffic_light.py` |
| DT-CMP-003 | Mission Controller | FSM, 차선 오차와 속도·조향 제어 | `turtlebot3_autorace_mission/control_lane.py` |
| DT-CMP-004 | Dynamic Obstacle | Gazebo 보행자 시나리오 | `dynamic_obstacle_plugin/src/box_follower.cpp` |
| DT-CMP-005 | ArUco Detector | Marker ID, pose와 거리 추정 | `aruco_yolo/aruco_detector.py` |
| DT-CMP-006 | Manipulation | TF 변환과 MoveIt Pick-and-Place | `aruco_yolo`, manipulation packages |
| DT-CMP-007 | Monitor UI | 영상, 위치, 제어 상태와 로그 표시 | `pyqt_robot/` |

## 주요 인터페이스

| ID | Topic/Interface | Producer → Consumer | 데이터 |
| --- | --- | --- | --- |
| DT-IF-001 | `/detect/lane` | Lane Detector → Controller | 차선 중심 |
| DT-IF-002 | `/detect/lane_state` | Lane Detector → Controller | 차선 상태 |
| DT-IF-003 | `/detect/traffic_light` | Traffic Detector → FSM/UI | 신호 상태 |
| DT-IF-004 | `/camera/person_detected` | Person Detector → Controller/UI | 사람 감지 여부 |
| DT-IF-005 | `/debug/control_state` | Controller → UI | 조향, 오차, pitch |
| DT-IF-006 | `/cmd_vel` | Controller → TurtleBot3 | 선속도와 각속도 |
| DT-IF-007 | `/detected_markers` | ArUco Detector → Manipulation | Marker ID와 pose |

## 동작 상태

```text
WAIT_SIGNAL
  └─ green → LANE_FOLLOW
LANE_FOLLOW
  ├─ intersection sign → INTERSECTION
  ├─ person detected → STOP_FOR_PERSON
  ├─ ramp detected → RAMP_CONTROL
  └─ pickup zone → PICK_AND_PLACE
```

상태 전이는 인식 결과와 주행 구간 조건으로 결정하며, 각 상태는 허용된 속도와 조향 명령만 생성한다.

## 환경 분리

Gazebo는 교통 신호, 사람, 경사로와 주행 코스를 재현한다. 실제 환경은 압축 영상과 경량화된 차선 검출을 사용한다. 두 환경은 차선, 신호, 제어 상태와 `/cmd_vel` 인터페이스를 공유한다.

