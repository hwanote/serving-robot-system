# 매장 설정 생성 도구(site_builder) 기능 요약 — 프론트엔드 팀용

작성일 2026-10-16 · 상태: **초안(provisional)**. 이 문서의 형식은 제안이고 팀 간 확정 계약이 아니에요.

## 한 줄 요약
매장 지도 위에 사람이 점(테이블·픽업대·대기 위치)을 찍으면, 로봇이 다니는 길(waypoints)과 테이블(entities)·구역(zones) 설정 파일을 자동으로 만들어 주는 로컬 도구예요. 만든 길은 시뮬레이션에서 실제로 달려 보고, 좁은 곳은 사람이 승인해서 조금씩 조정할 수 있어요.

## 도구가 하는 일 (순서대로)
1. **라벨링**: 지도(`map.yaml`)를 화면에 띄우고 점을 찍어요. 점 종류는 `table`(서빙할 테이블), `pickup`(음료를 싣는 카운터), `standby`(대기·홈 위치) 세 가지예요.
2. **생성**: 지도에서 로봇이 다닐 수 있는 폭을 계산해 직선 복도(노드·엣지)를 뽑고, 점마다 정지 위치와 방향(yaw)을 정해요. 좁거나 어긋난 곳은 OK / WARN / FAIL로 표시해요.
3. **주행 시험(선택)**: 생성한 길을 시뮬레이션이나 실제 로봇이 양방향으로 달려 보고 칸마다 통과·주의·실패를 기록해요.
4. **조정(선택)**: 시험 결과로 "이 복도를 5cm 옆으로" 같은 제안을 만들어요. 제안은 **사람이 승인한 것만** 적용돼요.
5. **내보내기**: 결과를 `config/generated/`에 써요(상태 `provisional`). 공통 설정(`config/common`)에는 절대 쓰지 않아요.

## 입력: labels.json
```json
{
  "schema_version": 1,
  "map_revision": 1,
  "labels": [
    {"id": "table_01", "kind": "table", "x_m": -1.60, "y_m": 0.65,
     "top_size_m": {"x": 0.8, "y": 0.8},
     "chairs": [{"dx": -0.65, "dy": 0.0, "size_m": 0.45}],
     "aliases": []},
    {"id": "kitchen", "kind": "pickup", "x_m": -2.30, "y_m": 2.40},
    {"id": "standby_01", "kind": "standby", "x_m": -3.20, "y_m": -2.40}
  ]
}
```
- `x_m`, `y_m`: 지도 좌표계(map frame)의 미터 값이에요. 화면에서 찍은 픽셀은 `map.yaml`의 `origin`과 `resolution`으로 바꿔요.
- `id`: 영소문자로 시작하고 영소문자·숫자·`_`만 써요. 중복 불가예요.
- `standby`는 **최소 1개 필요**해요. `top_size_m`, `chairs`, `aliases`, `map_revision`은 생략할 수 있어요.
- `map_revision`이 지도와 다르면 거부해요(지도가 바뀌었으니 점을 다시 찍으라는 뜻).

## 출력 (`config/generated/`)
| 파일 | 내용 |
|---|---|
| `waypoints.yaml` | 노드(x, y), 엣지(`id`, `from`, `to`, `zone`), 정지 위치(`poses`: 노드, `yaw_deg`, 종류, 바라보는 대상, 별칭) |
| `entities.yaml` | 로봇(`robot_01`), 테이블, 고정물 |
| `zones.yaml` | 구역 사각형 |
| `preview.json` | 화면 표시용: 노드·엣지별 검사 결과, 제외된 목적지와 경고, 요약(OK/WARN/FAIL 개수), 적용된 조정 |
| `labels.json` | 내보낼 때의 라벨 사본 |
| `adjustments_applied.json` | 적용한 조정 기록(조정 없이 내보내면 지워짐) |

모든 YAML 맨 위에 `GENERATED … status: provisional`이 붙어요. 검사는 `python3 scripts/validate_site_config.py --config-dir <폴더>`로 해요.

## 화면과 서버 API (로컬 전용)
실행: `python3 -m site_builder.server` → `http://127.0.0.1:8765/`

| 메서드·경로 | 하는 일 |
|---|---|
| `GET /api/map`, `/api/map.png` | 지도 크기·해상도·원점·revision / 지도 그림 |
| `GET /api/labels`, `PUT /api/labels` | 저장된 라벨 읽기 / 저장(대기 점이 아직 없는 작업 중 상태도 저장됨) |
| `GET /api/example` | 데모 매장 예시 라벨 |
| `POST /api/generate` | `{labels}` → `preview.json` 내용(아무것도 쓰지 않음) |
| `POST /api/export` | `{labels}` → 파일들을 쓰기(FAIL이 있으면 막힘) |
| `GET /api/drive` | 주행 시험 결과(복도별 양방향 판정), 조정 제안, 낡은 제안 여부 |
| `POST /api/drive/propose` | 시험 결과로 조정 제안 만들기(상태 `proposed`) |
| `PUT /api/drive/decide` | `{decisions: {adj01: approved/rejected/proposed}}` |

`generate`와 `export`는 `"adjusted": true`를 같이 보내면 승인한 조정을 적용해요.

**보안 규칙**: 127.0.0.1에서만 열려요. Host 헤더를 검사하고, JSON만 받아요. 공통 설정 폴더(`common`)로는 내보내지 않아요.

## 현재 상태 (데모 매장 기준)
- 노드 14개, 복도 16개(양방향 32칸)를 모의 주행해서 **전부 통과, 실패 0**이었어요(시뮬레이션, 사람 제한 영향 경고 일부 있음).
- 조정 제안 2건 중 1건(e02를 5cm 이동)을 승인·내보냈어요.
- **실제 로봇에서는 아직 시험하지 않았어요.**

## 프론트엔드 팀에 묻고 싶은 것
1. **테이블 번호를 어떻게 넘겨 주나요?** 지금 라벨 `id`는 `table_01` 같은 영문 ID이고, 다른 이름은 `aliases`에 넣을 수 있어요. 주문 화면의 책상 번호와 어떻게 연결할지 정해야 해요.
2. **매핑(지도 만들기) 때의 이동 경로를 저장하나요?** 저장한다면 주행 시험 경로의 참고 자료로 쓸 수 있어요.
3. 라벨 입력 화면을 이 도구와 합칠지, 이 도구가 만든 `labels.json`을 읽기만 할지 정해야 해요.

## 한계
- 지도(SLAM)를 다시 만드는 일은 범위 밖이에요.
- 직선 복도만 뽑아요(곡선 경로 없음). 조정은 한 라운드만 지원해요.
- 사람 감지 속도 제한은 시뮬레이션에서 오검출이 있어요(로봇 쪽 문제, 이 도구와 무관).
