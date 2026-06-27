# V-Model Development Process

## 적용 범위

TurtleBot3의 차선 주행, 교통 신호·표지판 인식, 사람 대응, 경사로 속도 보정, ArUco 기반 Pick-and-Place와 PyQt 모니터링을 하나의 자율주행 시스템으로 정의한다. Gazebo 디지털 트윈과 실제 로봇에 동일한 요구사항과 시나리오 ID를 적용한다.

이 문서는 Automotive SPICE 4.0의 SYS/SWE 프로세스와 지원 프로세스 산출물 구조를 참고한다. 규격 인증 또는 공식 준수를 의미하지 않는다.

## V-모델 매핑

| 개발 단계 | 산출물 | 대응 검증 | 근거 |
| --- | --- | --- | --- |
| 사용자 시나리오 | [requirements.md](requirements.md) STK 항목 | 실제 환경 검증 | 실제 TurtleBot 주행 영상 |
| 시스템 요구사항 | [requirements.md](requirements.md) SYS 항목 | Gazebo 시스템 시험 | 전체 코스 시뮬레이션 영상 |
| 시스템 아키텍처 | [architecture.md](architecture.md) | ROS2 통합 시험 | Topic, TF, MoveIt 연동 |
| 소프트웨어 요구사항 | [requirements.md](requirements.md) SWR 항목 | 노드·알고리즘 시험 | 차선, SIFT, ArUco 출력 |
| 상세 설계와 구현 | ROS2 Python/C++ 노드 | 단위 검증 | 영상 처리와 제어 함수 시험 |

## 시스템 경계

```text
Camera/IMU
   |
Perception Nodes ── lane/signal/person/marker
   |
Mission FSM and Lane Controller ── /cmd_vel ── TurtleBot3
   |
Gazebo or Physical Robot

ArUco Pose ── TF/MoveIt ── Manipulator
ROS2 Topics ── PyQt Monitor
```

## 산출물

- [요구사항 명세](requirements.md)
- [시스템 및 소프트웨어 아키텍처](architecture.md)
- [검증 및 확인 명세](verification-validation.md)
- [요구사항 추적성 매트릭스](traceability-matrix.md)
- [지원 프로세스](supporting-processes.md)

