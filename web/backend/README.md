# serving-robot backend (web/backend)

프론트엔드(`web/frontend`)와 로봇(`mission_cmd` / `mission_status`) 사이의 백엔드입니다.
FastAPI + SQLAlchemy(SQLite, URL만 바꾸면 PostgreSQL). API 는 `docs/API.md`.

## 구조
- `app/routers/` HTTP·WebSocket 프로토콜만 (검증 결과를 응답으로 바꿈)
- `app/services/` 입력 검증, 주문 생성·취소
- `app/fleet/` 대기열·우선순위, 구역(zone), 경로 겹침 검사, 주방 잠금, 유휴 홈 복귀, 하트비트, KPI
- `app/ros_adapter/` 로봇별 어댑터 (`ros` 실제, `fake` 가짜). 팀 Fleet 모듈로 바꿀 때 여기만 교체
- `app/repository/` 저장소 (`memory`, `sql`). 재시작 후에도 기록이 남음 (`data/backend.db`)
- `app/semantics.py` 지도 설정(Semantic Map) 초안 저장과 자동 노드 적용 (`tools/site_builder` 사용, 결과는 `data/`)
- `config/fleet.example.yaml` 로봇·구역·우선순위·시간 설정 예시

## 운영 모드 (`--mode`)
| 모드 | 동작 |
|---|---|
| A | 로봇 1대 (목록의 첫 로봇), 겹침 검사 없음 |
| B | 여러 대, 가장 가까운 빈 로봇이 가장 오래된 주문을 받음 (주방 잠금만) |
| C | 여러 대 협업: 구역 + 경로 겹침 검사 + 도움 (기본값) |

## 실행 (WSL)
```bash
# Windows 에서 고친 파일을 WSL 로 복사 (처음 한 번, 이후 수정할 때마다)
bash /mnt/c/Users/user/Desktop/serving-robot-system/web/backend/scripts/sync_to_wsl.sh
cd ~/serving-robot-system/web/backend
cp config/fleet.example.yaml config/fleet.yaml     # 필요하면 수정
scripts/run_wsl.sh --adapter ros --config config/fleet.yaml     # 실제 로봇(ROS)
scripts/run_wsl.sh --adapter fake --robots robot_01,robot_02    # 가짜 로봇
```
우선순위: 기본값 < `--config` 파일 < 명령줄. 옵션 전체는 `--help`.
로봇 쪽(Gazebo/navigation) 터미널은 기존과 동일하고, 프론트는 `web/frontend` 에서 `npm run dev:real`.

## 테스트
```bash
pip install -r requirements-dev.txt && python -m pytest -q tests     # web/backend 에서, 약 1분
```
