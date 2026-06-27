# Supporting Processes

## 형상관리

| 형상 항목 | 식별 정보 |
| --- | --- |
| ROS2 source | package, 파일 경로와 Git commit |
| Launch/parameter | 실행 환경, YAML과 launch 파일 hash |
| 인식 자료 | SIFT template, ArUco calibration, 모델 파일 |
| Gazebo 환경 | world, model, plugin 기준선 |
| 시험 자료 | 시나리오 ID, ROS bag, 로그, 영상 |
| 실제 로봇 환경 | TurtleBot firmware, camera 설정, ROS2 Humble |

Gazebo와 실제 로봇 기준선은 별도로 식별하되 공통 요구사항과 interface ID를 사용한다.

## 변경관리

변경 요청은 ID, 변경 목적, 대상 package, 영향받는 요구사항과 topic, Gazebo/실기 영향, 회귀 시험, 검토와 승인 정보를 포함한다. parameter 변경도 source 변경과 동일하게 기록한다.

## 문제관리

다음 문제는 재현 시나리오와 rosbag 또는 영상 근거를 포함한다.

- 잘못된 신호 또는 표지판 분류
- 차선 중심 급변과 과도한 조향
- 사람 감지 중 이동 명령 발생
- topic 단절 또는 stale data 사용
- TF 불일치로 인한 잘못된 Pick-and-Place pose
- UI 갱신 지연이 ROS2 callback을 방해하는 현상

## 도구와 데이터

- Ubuntu 22.04, ROS2 Humble
- Gazebo, TurtleBot3 packages
- Python, C++, OpenCV, SIFT, PyQt
- ArUco calibration data, MoveIt configuration

도구 버전, calibration 파일과 parameter는 시험 결과와 함께 식별한다.

## 검토 체크리스트

- 각 상태 전이에 입력 조건과 허용 출력이 정의되어 있는가?
- perception 결과가 stale 또는 invalid일 때 안전한 출력이 정의되어 있는가?
- Gazebo와 실제 환경의 topic, 단위와 좌표계가 일치하는가?
- 모든 요구사항에 노드 시험 또는 시스템 시나리오가 연결되는가?
- 영상과 로그가 사용한 source와 parameter 기준선을 식별하는가?

