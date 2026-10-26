#!/bin/bash
# Windows 폴더의 web/backend 와 tools/site_builder 를 WSL 의 ~/serving-robot-system 아래로 복사하고 비교 (data/ 와 캐시는 제외)
S=/mnt/c/Users/user/Desktop/serving-robot-system/web/backend
D=$HOME/serving-robot-system/web/backend
mkdir -p "$D"
( cd "$S" && tar --exclude=data --exclude=fleet.yaml --exclude=__pycache__ --exclude=.pytest_cache -cf - . ) | ( cd "$D" && tar -xf - )
if diff -rq -x data -x fleet.yaml -x __pycache__ -x .pytest_cache "$S" "$D"; then echo "same: web/backend"; else echo "DIFF 있음"; fi
# 지도 설정 '적용 저장' 이 불러 쓰는 생성기(tools/site_builder) 도 같이 복사 (git pull 을 안 해도 최신 상태가 됨)
SB_S=/mnt/c/Users/user/Desktop/serving-robot-system/tools/site_builder
SB_D=$HOME/serving-robot-system/tools/site_builder
mkdir -p "$SB_D"
( cd "$SB_S" && tar --exclude=__pycache__ --exclude=.pytest_cache -cf - . ) | ( cd "$SB_D" && tar -xf - )
if diff -rq -x __pycache__ -x .pytest_cache "$SB_S" "$SB_D"; then echo "same: tools/site_builder"; else echo "DIFF 있음 (tools/site_builder)"; fi
