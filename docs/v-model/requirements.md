# Requirements Specification

## 이해관계자 요구사항

| ID | 요구사항 | 확인 기준 |
| --- | --- | --- |
| DT-STK-001 | 로봇은 직선과 곡선 차선을 따라 코스를 주행해야 한다. | 차선 이탈 없이 지정 구간을 통과한다. |
| DT-STK-002 | 로봇은 교통 신호와 표지판에 맞는 주행 행동을 선택해야 한다. | 정지, 출발, 회전과 진입 회피 상태가 시나리오와 일치한다. |
| DT-STK-003 | 횡단보도에서 사람이 감지되면 안전하게 정지해야 한다. | 사람 감지 중 정지하고 해제 조건에서 주행을 재개한다. |
| DT-STK-004 | 경사로에서도 안정적인 속도로 주행해야 한다. | IMU pitch에 따라 오르막과 내리막 속도가 보정된다. |
| DT-STK-005 | 지정된 ArUco 물체의 위치를 추정하고 Pick-and-Place를 수행해야 한다. | 마커 pose가 로봇 좌표로 변환되고 조작 명령이 실행된다. |
| DT-STK-006 | 운영자는 주행 상태와 인식 이벤트를 실시간으로 확인할 수 있어야 한다. | 카메라, 미니맵, 속도, pitch와 이벤트가 UI에 표시된다. |

## 시스템 요구사항

| ID | 시스템 요구사항 | 할당 | 판정 기준 |
| --- | --- | --- | --- |
| DT-SYS-001 | 시스템은 카메라 영상에서 좌우 차선과 차선 중심을 계산해야 한다. | Lane Detection | `/detect/lane`, `/detect/lane_state` 출력 확인 |
| DT-SYS-002 | 시스템은 등록된 교통 신호 템플릿과 입력 영상의 특징점을 비교해야 한다. | Traffic Detection | `/detect/traffic_light` 상태 확인 |
| DT-SYS-003 | 시스템은 인식 이벤트와 주행 구간을 FSM 상태로 관리해야 한다. | Mission Control | 상태에 맞는 속도와 조향 명령 확인 |
| DT-SYS-004 | 사람 감지 상태에서는 이동 명령을 정지 상태로 설정해야 한다. | Person Detection/Control | 감지 토픽 활성 시 선속도 0 확인 |
| DT-SYS-005 | 시스템은 IMU pitch에 따라 경사 방향을 판별하고 속도를 보정해야 한다. | Lane Control | pitch 부호와 속도 변화 방향 일치 |
| DT-SYS-006 | 시스템은 ArUco ID와 3D pose를 발행하고 로봇팔 목표 pose로 변환해야 한다. | ArUco/TF/MoveIt | `/detected_markers`와 조작 목표 확인 |
| DT-SYS-007 | 시뮬레이션과 실제 환경은 동일한 주요 ROS2 인터페이스를 사용해야 한다. | ROS2 Integration | 핵심 topic 이름과 message type 일치 |
| DT-SYS-008 | UI는 카메라, 위치, 속도, 제어 오차, pitch와 이벤트를 구독해 표시해야 한다. | PyQt | 각 화면 요소와 topic 갱신 확인 |

## 소프트웨어 요구사항

| ID | 소프트웨어 요구사항 | 판정 기준 |
| --- | --- | --- |
| DT-SWR-001 | 차선 검출은 이진화와 morphology 후 contour를 추출해야 한다. | 중간 영상과 contour 출력 확인 |
| DT-SWR-002 | 큰 contour와 곡선 조건에서는 분할 또는 sliding window 방식을 적용해야 한다. | H자·곡선 입력에서 좌우 차선을 분리한다. |
| DT-SWR-003 | 신호 인식은 SIFT descriptor와 BFMatcher 결과로 템플릿을 선택해야 한다. | 기준 템플릿별 매칭 결과 확인 |
| DT-SWR-004 | 차선 중심 오차로 각속도를 계산하고 오차 크기에 따라 선속도를 제한해야 한다. | 오차 증가 시 선속도 감소와 조향 증가 확인 |
| DT-SWR-005 | 제어 출력은 TurtleBot3 허용 속도 범위로 제한되어야 한다. | 최대 선속도·각속도 경계 확인 |
| DT-SWR-006 | ArUco 검출은 camera matrix와 distortion coefficient로 marker pose를 계산해야 한다. | 기준 거리와 pose 오차 기록 |
| DT-SWR-007 | UI 갱신은 ROS2 callback과 Qt signal을 분리해야 한다. | callback 수신 중 UI 응답 유지 |

