# Robot 01 자동 매핑 + 지도 사용자 보정 모듈 전달

> **Draft PR 준비용 계약서입니다. ROS/Gazebo 실동작 및 소스 반영 전에는 병합하지 마세요.**

## 목표
기존 팀 UI의 `Start Mapping → Stop Mapping → 사용자 지도 보정 → Save Map` 흐름에 매핑팀 모듈을 붙입니다. 독립적인 지도 설정 페이지, 테이블/정차 영역 자동 계산, 서빙 경로, Robot 02, Fleet 로직은 납품 범위에서 제외합니다.

## 책임 분리
- **매핑 파트**: Robot 01 SLAM/Frontier 기반 자동 탐색, 중지·종료 판정, 임시 PGM/YAML 생성, 추천 수정/직선/자유곡선/개별 삭제/되돌리기, 최종 PGM/YAML 보존 저장
- **프론트엔드 파트**: 기존 `MappingPanel` 버튼과 `MapReviewPanel` 컴포넌트를 팀 UI에 배치 및 연결
- **자율주행 파트**: 저장된 최종 `navigation_map.pgm` / `navigation_map.yaml`를 자신의 Nav2/AMCL 런타임에 적용하고 검증

## 기존 코드에서 확인된 연결점
- ROS 원본 (WSL **미커밋 작업 포함**): `ros_ws/src/serving_robot_mapping`; 콘솔 진입점 `exploration_manager`.
- 세션 실행기: `tools/run_lidar_mapping_session.py --run --log-dir <session-dir>`.
- 종료 상태 정책: `serving_robot_mapping/map_draft_policy.py`; `COVERAGE_LIMITED`는 완전 매핑의 보증이 아님.
- 프론트: `MappingPanel.tsx` → `useMappingControl.ts` → `sendMappingCommand`.
- 기존 FastAPI `/api/mapping/{command}`는 501 미구현 상태이므로 교체 필요.
- 보정: `map_review.py`, `MapReviewPanel.tsx`, `MapViewport.tsx`.

## 매핑 API 계약
| Method | Path | 의도 |
|---|---|---|
| POST | `/api/mapping/start` | Robot 01 자동 매핑 세션 시작 |
| GET | `/api/mapping/status` | 시작 시간, 상태, Frontier 종료 이유, 임시 지도 준비 여부 |
| POST | `/api/mapping/stop` | **이 API가 시작한** 세션만 취소/정리, 임시 지도 생성 확인 |
| GET | `/api/stores/{store_id}/map-review` | 임시 지도 메타데이터, 보정 추천, 원본 해시 |
| GET | `/api/stores/{store_id}/map-review.png` | 임시 지도 미리보기 |
| POST | `/api/mapping/save` | `source_hash`, `confirmed=true`, `strokes` 확인 후 최종 저장 |

`Stop`은 임시 지도 생성까지 비동기일 수 있습니다. 프론트는 `draft_ready` 확인 전 보정을 시작할 수 없습니다. 사용자 최종 확인 전 `Save`는 차단합니다. `save` 호출이 성공하면 프론트가 새 지도를 갱신해야 합니다.

보정선 예시:
```json
{"source_hash":"<sha256>", "confirmed":true, "strokes":[{"x1":1,"y1":2,"x2":3,"y2":4,"value":100}]}
```
`value`는 보정 모듈에서 0(자유 공간), 100(벽), 255(미확인)을 의미하며 자유 공간 수정은 사용자 확인 후 적용합니다.

## 저장 산출물
- 매핑 원본 PGM/YAML: 세션별 보존, 덮어쓰기 금지
- 보정 후 `navigation_map.pgm`, `navigation_map.yaml`, `review_record.json` 저장
- 원본/최종 해시 기록; Unknown을 임의로 자유 공간으로 변경하지 않기
- UI 사용자의 지도 저장은 **실제 Nav2 운행 승인과 별개**

## 현재 상태 (2026-10-26)
- [x] WSL 납품 폴더로 매핑 ROS 패키지 81개 소스 복사 — **사용자 실행 출력 기준**
- [x] 매핑 지도 → 보정 입력 → PGM/YAML 출력 어댑터 샘플 테스트 — **사용자 실행 출력 기준**
- [x] FastAPI 매핑 API 안전 차단/구문 테스트 — **사용자 실행 출력 기준**
- [ ] **이 PR 브랜치에 WSL 실제 매핑 소스·보정 소스 업로드** (현재 이 PR에는 없음)
- [ ] `MapReviewPanel`의 최종 저장 요청을 기존 `Save Map` 하나로 연결
- [ ] `Stop` 후 비동기 세션 종료·자식 Nav2 취소·임시 맵 판정 검증
- [ ] 저장 후 팀 프론트 지도 새로고침과 데이터 모델 연동
- [ ] Gazebo Robot 01 `Start → Stop → 보정 → Save` E2E 실동작
- [ ] GitHub PR 소스·테스트 결과 최종 검토

## 주의
원본 WSL Mapping 브랜치는 미커밋 변경이 있으므로 이 PR 때문에 `git checkout/reset/clean`하지 마세요. **별도 작업 폴더 또는 `git worktree`에서 파일을 가져와 커밋**하세요. ROS 운행/프로세스 제어 검증 전에는 실제 실행을 활성화하지 마세요.
