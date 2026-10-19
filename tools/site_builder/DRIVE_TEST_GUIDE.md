# 주행 시험 실행 안내 (VM에서 실행)

로봇 쪽(ROS 2 + Gazebo)과 주행 시험 도구는 **VM**에서 실행해요. 시뮬레이션에서는 2026-10-16에 32칸을 모두 통과해 본 순서예요. 오류가 나면 출력을 그대로 붙여 주세요.

모든 터미널의 맨 위에는 아래 세 줄이 필요해요(아래 블록마다 반복해서 적었어요).
```bash
source /opt/ros/jazzy/setup.bash
source ~/serving-robot-system/ros_ws/install/setup.bash
export SERVING_ROBOT_CONFIG_DIR=/home/ubuntu/serving-robot-system/tools/site_builder/out/drive_demo
```

## 0. 준비
PC에서 고친 파일을 VM으로 옮겨요(결과가 `일치 N / 다름 0`이면 같은 거예요).
```bash
bash /mnt/hgfs/serving-robot-system/scripts/dev/vm_sync.sh
```
시험용 폴더를 만들어요(한 번). `out/`은 동기화 대상이 아니라 VM에만 생겨요.
```bash
cd ~/serving-robot-system/tools/site_builder
python3 -m site_builder.cli generate --map ../../maps/stores/demo_store/map.yaml \
    --labels examples/labels_demo.json --out-dir out/gen_demo
python3 -m site_builder.drive plan --candidate out/gen_demo --out-dir out/drive_demo
```
`plan`은 `out/drive_demo`와 `tour.json`(32칸, 예상 약 5분)을 만들어요. `config/common`은 건드리지 않아요.

## 1. 로봇 쪽 실행 (터미널 A)
```bash
source /opt/ros/jazzy/setup.bash
source ~/serving-robot-system/ros_ws/install/setup.bash
export SERVING_ROBOT_CONFIG_DIR=/home/ubuntu/serving-robot-system/tools/site_builder/out/drive_demo
ros2 launch serving_robot_bringup operation_mode.launch.py person_limit:=false
```
- `person_limit:=false`는 **시뮬레이션 시험 전용**이에요. 사람 모델이 없는데도 사람으로 감지되는 문제가 있어 속도 제한이 걸리기 때문이에요. **실제 로봇에서는 반드시 켠 채로** 써요(옵션을 빼면 기본값이 켜짐이에요).
- 켠 뒤 다른 터미널에서 `ps aux | grep -E "person_speed_limiter|person_detector" | grep -v grep` 결과가 비어 있어야 꺼진 거예요.

## 2. 주행 (터미널 B)
```bash
source /opt/ros/jazzy/setup.bash
source ~/serving-robot-system/ros_ws/install/setup.bash
export SERVING_ROBOT_CONFIG_DIR=/home/ubuntu/serving-robot-system/tools/site_builder/out/drive_demo
ros2 daemon stop
ros2 daemon start
cd ~/serving-robot-system/tools/site_builder
```
(`ros2 daemon` 두 줄은 터미널 사이에 노드가 안 보일 때 필요해요.)

먼저 한 구간만:
```bash
python3 -m site_builder.drive run --tour out/drive_demo/tour.json --mode sim --time-scale 3.5 --only e01
```
이상 없으면 전체:
```bash
python3 -m site_builder.drive run --tour out/drive_demo/tour.json --mode sim --time-scale 3.5
```
- `--time-scale`은 예상 시간에 곱하는 배율이에요(시뮬레이션이 실시간보다 느려서). 시간 초과 판정은 `예상×배율×3 + 10초`예요. 실물 로봇에서는 1.0(기본)을 써요.
- 끊겨도 `out/drive_demo/test_report.json`에 칸마다 저장돼요. **같은 명령을 다시 실행하면 이어서** 해요. 실패로 끝난 칸도 "한 칸 끝남"으로 치니, 그 칸만 다시 하려면 새 보고서 파일을 써요(아래).
- 막힘/HardStop/충돌이 나면 그 칸은 바로 취소하고 기록한 뒤 다음 칸으로 가요(우회 없음). 4칸 연속 실패하면 자동으로 멈춰요.
- 칸이 이어지지 않아도(`--only`) 로봇이 시작 노드에 없으면 알아서 옮겨요(시뮬레이션은 Gazebo에서 순간이동, 옮긴 뒤 위치추정이 그 노드 30 cm 이내인지 확인해요).

### 일부 칸만 다시 하기
```bash
python3 -m site_builder.drive run --tour out/drive_demo/tour.json --report out/drive_demo/retry_run.json \
    --mode sim --time-scale 3.5 --only 10 11 12
```
보고서 파일을 새로 지정하면 처음부터 새로 기록해요. 끝나면 두 보고서를 합쳐서(예전 칸 + 새로 한 칸) 한 파일로 만들어요.
```bash
python3 - <<'PY'
import json
d = 'out/drive_demo/'
full = json.load(open(d + 'full_run.json'))      # 처음 기록
retry = json.load(open(d + 'retry_run.json'))    # 다시 한 기록
merged = dict(retry)
merged['steps'] = {k: v for k, v in full['steps'].items() if int(k) < 10}   # 다시 하지 않은 칸만 남김
merged['steps'].update(retry['steps'])
json.dump(merged, open(d + 'merged_run.json', 'w'), indent=1, ensure_ascii=False)
print('합친 단계 수:', len(merged['steps']))
PY
```
(`int(k) < 10`은 예시예요. 다시 한 칸 번호에 맞게 바꿔요.)

### 결과 표
```bash
python3 -m site_builder.drive summary --tour out/drive_demo/tour.json --report out/drive_demo/merged_run.json
```
`OK`는 문제 없음, `WARN`은 통과했지만 표시가 있음(`STALLED` 3초 이상 멈춤, `PERSON_LIMIT` 사람 제한 1초 이상, `TIGHT` 좁음, `SLOW` 느림, `HARDSTOP_BRIEF` 잠깐 HardStop), `FAIL`은 못 감이에요.

## 3. 조정 제안과 승인
화면은 보고서 이름이 `test_report.json`이어야 읽어요. 합친 결과를 그 이름으로 복사한 뒤 제안을 만들어요(예전 파일이 있으면 이름을 바꿔 보관).
```bash
cd ~/serving-robot-system/tools/site_builder
[ -f out/drive_demo/test_report.json ] && mv out/drive_demo/test_report.json out/drive_demo/test_report_old.json
cp out/drive_demo/merged_run.json out/drive_demo/test_report.json
python3 -m site_builder.drive adjust --tour out/drive_demo/tour.json --report out/drive_demo/test_report.json \
    --out out/drive_demo/adjustments.json
```
출력 종류: `shift_line`(줄을 옆으로 이동, 최대 ±0.15 m, 가장 좁은 곳 여유가 3 cm 이상 늘 때만), `remove_edge`(양방향 모두 실패한 복도 제거), `review_edge`(사람이 확인, 자동 적용 안 됨).

그다음 화면을 열어요.
```bash
python3 -m site_builder.server
```
VM 브라우저에서 `http://127.0.0.1:8765/` → **「예시 불러오기」**(점이 비어 있으면 필수) → 3번 카드에서 결과 확인 → 「조정 제안 만들기」 → 승인/거절 → 「승인한 조정 적용해서 보기」(원래 구간은 회색 점선) → 마음에 들면 「조정본 내보내기」.
- 승인한 것만 적용돼요. 지도나 라벨이 시험 때와 달라졌으면 "낡은 제안"이라며 거부해요.
- FAIL이 하나라도 있으면 내보내기가 막혀요.
- 내보내기는 `config/generated/`에만 써요(상태 provisional). `config/common`으로는 절대 쓰지 않아요.
- 이동한 결과 이전보다 나빠지는 곳(OK였던 노드나 구간이 WARN이 되는 경우)이 생기면 그 제안은 적용되지 않고, 화면에 "못 한 것"과 이유가 나와요.
- 조정한 뒤에는 한 라운드만 지원해요(조정본을 다시 시험하면 새 제안 없이 결과만 보여줘요).

## 4. 실제 로봇 (나중에, 처음부터 사람이 옆에서)
```bash
python3 -m site_builder.drive run --tour out/drive_demo/tour.json --mode manual --only e01
```
- **비상정지를 손에 든 사람이 로봇 옆에 있어야 해요**(시작할 때 확인해요). 속도 제한은 `runtime_limits.yaml` 그대로예요.
- 터미널 A는 `person_limit:=false` 없이 켜요.
- 칸마다 "로봇을 노드 X에 놓고 Enter" 안내가 나와요(q = 중단).
- 시간 배율은 1.0이에요.

## 알려진 한계
- 사람 모델이 없는 시뮬레이션에서도 사람이 감지돼 속도 제한(SLOW 0.10 m/s, STOP)이 걸리는 문제가 있어요(원인 미조사).
- 칸마다 레일 이탈 최대치가 0.15 m 근처로 나오는데, 로봇 쪽 OFF_RAIL 경고 조건은 확인하지 못했어요.
