# Bidirectional Traceability Matrix

| Stakeholder | System | Software | Architecture/Code | Verification |
| --- | --- | --- | --- | --- |
| DT-STK-001 | DT-SYS-001, DT-SYS-003 | DT-SWR-001, DT-SWR-002, DT-SWR-004, DT-SWR-005 | DT-CMP-001/003 | DT-UT-001/003, DT-IT-001, DT-ST-001/002 |
| DT-STK-002 | DT-SYS-002, DT-SYS-003 | DT-SWR-003, DT-SWR-005 | DT-CMP-002/003 | DT-UT-002, DT-IT-002, DT-ST-001 |
| DT-STK-003 | DT-SYS-003, DT-SYS-004 | 정지 상태 요구사항 | DT-CMP-003/004 | DT-IT-003, DT-ST-003 |
| DT-STK-004 | DT-SYS-005 | DT-SWR-004/005 | DT-CMP-003, IMU interface | DT-UT-004, DT-IT-004, DT-ST-004 |
| DT-STK-005 | DT-SYS-006 | DT-SWR-006 | DT-CMP-005/006 | DT-UT-005, DT-IT-005, DT-ST-005 |
| DT-STK-006 | DT-SYS-008 | DT-SWR-007 | DT-CMP-007 | DT-IT-006, DT-ST-006 |
| DT-STK-001~006 | DT-SYS-007 | ROS2 interface requirements | DT-IF-001~007 | 전체 통합 시험 |

## 변경 영향 확인

- 영상 전처리 또는 threshold 변경: DT-UT-001, DT-IT-001, DT-ST-002 재검토
- SIFT template 변경: DT-UT-002, DT-IT-002, DT-ST-001 재검토
- FSM 전이 변경: 신호·사람·경사로 관련 통합 및 시스템 시험 재검토
- topic 또는 message type 변경: producer, consumer와 PyQt 인터페이스를 함께 검토
- 카메라 calibration 변경: ArUco pose와 Pick-and-Place 시험 재검토
