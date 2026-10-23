# TEAM 두 로봇 통합 실행 및 검증

기준 저장소: `hwanote/serving-robot-system`, 기준 커밋 `6d87bda`.
작업 브랜치: `integration/team-dual-robot-20261022`. 원격 push는 수행하지 않는다.

## 실행

WSL Ubuntu에서 기존 팀 저장소로 이동한다.

```bash
cd /home/user/team_serving_robot_clean_20261022
bash scripts/build.sh
bash scripts/start_operation.sh
```

화면 없이 시험하려면 마지막 명령에 `--headless`를 붙인다. 운영 화면은
`http://localhost:8765`이다. 실행 종료는 실행 터미널의 Ctrl+C 또는 다른 터미널에서
`bash scripts/stop.sh`를 사용한다. 관리 스크립트는 자기 프로세스 그룹만 종료한다.
기본 ROS Domain은 101, Gazebo partition은 `team_integration_101`이다.
환경 변수로 다른 Domain/partition을 지정할 수 있다. 같은 partition의 중복 실행은 거절한다.

설정은 `config/integration/`에서 관리한다. `system.yaml`은 저장소 상대 경로를 사용하며,
시작 시 `_local_only/config/system.yaml`에 실제 절대 경로를 생성한다.
속도/안전/오차/시간은 `runtime_limits.yaml`, 테이블 자세는 `waypoints.yaml`,
공유 구역/진입 금지/감속 구역은 `zones.yaml`에 있다. 변경 후 재시작한다.
Python 파일을 변경하면 `--symlink-install` 사용 여부와 관계없이 다시 빌드한다.
이번 환경의 Python 설치본은 소스와 별도 복사본이었다.

## 지도

원본 `maps/stores/demo_store/`의 세 파일은 기존처럼 0바이트이며 덮어쓰지 않았다.
LAB `cafe_clean_step96` 원본도 변경하지 않았다. 현재 운행 지도는 이 작업에서
**원본 카페 월드의 실제 LiDAR → SLAM Toolbox → map_saver**로 저장한
`_local_only/maps/lidar_initial.yaml`이다. SDF 충돌 형상으로 만든 지도가 아니다.
첫 로봇 spawn에서 odom/map 원점은 0,0이고 world 위치는 -3.2,-2.25이므로
현재 world → map 이동은 +3.2,+2.25, yaw 0이다. 지도 추정과 실제 위치의
차이는 독립적으로 계측해야 하며, 이 변환 자체가 물리 도착 정확도를 보증하지 않는다.

Mapping은 Operation을 먼저 종료한 다음 실행한다.

```bash
bash scripts/start_mapping.sh
# 별도 터미널에서 저장. 동일 이름 덮어쓰기는 거절한다.
bash scripts/save_map.sh cafe_new_scan
```

Mapping에는 Robot 01과 SLAM만 실행하며 AMCL/Nav2 운행 스택은 함께 실행하지 않는다.
현재 Mapping Launch는 센서 취득·지도 저장용이다. 안전한 지도 확장 주행 조작기는
아직 연결하지 않았으므로 이 모드에서 base 토픽에 임의 속도를 발행하지 않는다.
새 clone에는 `_local_only`가 포함되지 않으므로 유효한 SLAM 지도와 가중치를 별도로
준비해야 한다. 가중치는 `bash scripts/fetch_models.sh`로 준비할 수 있다.
Python YOLO 의존성은 현재 WSL의 `ultralytics`/PyTorch 설치를 사용한다.

## 미션과 운영 API

대시보드에서 자동 배차 또는 로봇을 지정하고 테이블과 미션 방식을 선택한다.
`KITCHEN_PICKUP`: 부엌 이동 → WAIT_LOAD에서 적재 확인 → 테이블 이동 →
WAIT_HANDOVER에서 서빙 확인 → HOME 이동 → COMPLETED 순서이다.
Nav2 도착만으로 서빙 완료를 처리하지 않는다. ERROR/CANCELED 상태는 선택 로봇
RESET 후 다시 요청할 수 있다. TABLE_COLLECT는 회수 완료 입력을 사용한다.

- `GET /api/status`: 미션·Safety·map 위치·Zone owner·복구 상태.
- `GET /api/history`: SQLite 운영 이력.
- `POST /api/request`: SUBMIT, LOAD, COMPLETE, COLLECTED, CANCEL, RESET.
- SUBMIT 예: `{"action":"SUBMIT","request_id":"order-001","table":"table_03","robot_id":"robot_01","mode":"KITCHEN_PICKUP"}`.
- 적재 확인: `{"action":"LOAD","request_id":"order-001","source":"kitchen"}`.
- 서빙 확인: `{"action":"COMPLETE","request_id":"order-001","source":"table"}`.

## 실제 시험 도구

시스템 실행 후 다음 도구는 실제 Nav2/Fleet를 사용한다. 시험 도구의 적재·서빙 입력은
운영자 입력을 자동으로 모사한다. 사람 검출값이나 성공 상태를 주입하지 않는다.

```bash
python3 scripts/trial_serving.py --timeout 300
python3 scripts/trial_serving.py --mode KITCHEN_PICKUP --timeout 600
python3 scripts/trial_serving.py --cross --timeout 600
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=101 GZ_PARTITION=team_integration_101
python3 scripts/probe_runtime.py
python3 scripts/person_scenario.py spawn --relative-to robot_02 --distance 1.3
python3 scripts/human_acceptance.py
python3 scripts/person_scenario.py clear
```

사람 배치 도구는 실제 Gazebo 모델의 현재 자세를 읽어 상대 위치를 계산한다.
벽/가구 밖인지 판단하여 배치해야 한다. `move --x ... --y ... --to-x ... --to-y ...`
옵션으로 시뮬레이터의 사람 모델을 이동할 수 있다. 검출기는 그 모델의 좌표를 읽지 않고
실제 RGB·등록 depth·카메라 내부 파라미터·TF로 사람 위치를 계산한다.
YOLO 결과가 중단되면 Safety는 정지한다. LiDAR와 Fleet 하트비트도 동일하게 감시한다.

속도 흐름은 Nav2 → velocity smoother → Semantic Safety → Collision Monitor →
Robot Base이다. 정밀 정렬도 Semantic Safety 입력으로만 들어간다.
`probe_runtime.py`는 base 속도 토픽의 publisher가 Collision Monitor 하나인지 확인한다.

## Git에 올리지 않는 폴더

`/_local_only/` 하나에 다음 로컬 자료를 모았다.

- `archive/unused_imported_modules`, `archive/unused_imported_config`: 실제 미사용 가져온 코드·중복 설정.
- `evidence/`: 이전 빌드/실행 로그, 원본 보존 해시, 개발본 diff, Fleet DB.
- `maps/`, `models/`: 실제 운행에 필요한 로컬 지도·YOLO 가중치. 이들은 **사용 중**이지만 Git 제외 대상이다.
- `metrics/`, 실행 로그, 시험 JSON/JSONL: 재현 및 회귀 검증 증거.
- `cleanup_manifest.json`: 이동한 파일의 원래 위치와 새 위치.

원본 Git 추적 소스·월드·지도는 정리 대상으로 이동하지 않았다.
`git check-ignore`로 제외를 확인했으며 `_local_only`에서 추적 중인 파일은 없다.

## 아직 완료 판정하면 안 되는 항목

현재 검증 결과는 `TEAM_VALIDATION_KO.md`에 기록한다. 지도 추정 도착과 Gazebo 실제
위치 오차를 구분해야 한다. 모든 테이블·접근 방향·사람 보행·No-Go 진입 거절·
반복 신뢰성 시험이 통과하기 전에는 전체 요구사항 완료로 선언하지 않는다.
