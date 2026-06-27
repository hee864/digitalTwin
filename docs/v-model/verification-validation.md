# Verification and Validation

## 단위 및 노드 검증

| ID | 대상 | 입력/조건 | 판정 기준 |
| --- | --- | --- | --- |
| DT-UT-001 | Lane preprocessing | 직선·곡선·H자 차선 이미지 | 좌우 차선과 중심값 산출 |
| DT-UT-002 | SIFT recognition | 등록 신호 템플릿의 조도·각도·크기 변형 | 올바른 신호 label 선택 |
| DT-UT-003 | Lane controller | 중심 오차 0, 양수, 음수 | 조향 방향과 속도 변화가 기준식과 일치 |
| DT-UT-004 | Ramp controller | pitch 음수, 0, 양수 | 오르막·평지·내리막 속도 정책 일치 |
| DT-UT-005 | ArUco pose | ID와 거리가 알려진 marker | ID, tvec, distance 오차 기록 |

## ROS2 통합 시험

| ID | 결합 대상 | 확인 항목 | 관찰 지점 |
| --- | --- | --- | --- |
| DT-IT-001 | Lane Detector ↔ Controller | 차선 중심에 따른 `/cmd_vel` | lane/control topics |
| DT-IT-002 | Traffic Detector ↔ FSM | 신호별 정지·출발 상태 | traffic/FSM topics |
| DT-IT-003 | Person Detector ↔ Controller | 감지 시 정지와 해제 후 출발 | person/cmd_vel topics |
| DT-IT-004 | IMU ↔ Controller | pitch에 따른 속도 보정 | IMU/debug/cmd_vel |
| DT-IT-005 | ArUco ↔ MoveIt | marker pose의 로봇 목표 변환 | MarkerArray, TF, MoveIt |
| DT-IT-006 | ROS2 Nodes ↔ PyQt | 데이터 수신 중 화면 갱신 | 카메라, 그래프, 로그 |

## 시스템 검증

| ID | 시나리오 | 판정 항목 | 근거 |
| --- | --- | --- | --- |
| DT-ST-001 | Gazebo 전체 코스 | 신호, 교차로, 사람, 경사로 구간 완주 | [Gazebo 영상](https://youtu.be/eU3tc6r3l6o), `image/course.png` |
| DT-ST-002 | 실제 차선 주행 | 직선·곡선·H자 차선 추종 | [실제 주행 영상](https://youtu.be/t-657SxmHgo), `image/straight.png`, `image/curve.png` |
| DT-ST-003 | 사람 대응 | 감지 중 정지와 안전 조건에서 재출발 | `image/person.gif` |
| DT-ST-004 | 경사로 주행 | pitch와 선속도 변화 관계 확인 | `image/ramp.gif`, `image/8.png` |
| DT-ST-005 | Pick-and-Place | 마커 인식, 좌표 변환과 pick 동작 | `image/aruco_pick.gif`, `image/9.jpg` |
| DT-ST-006 | 운영 모니터링 | 카메라, 미니맵, 속도, pitch, 이벤트 표시 | `image/6.png` |

시험 기록에는 ROS bag 또는 로그, parameter 파일, source commit, 환경 구분, 기대 결과와 실제 결과를 포함한다.

