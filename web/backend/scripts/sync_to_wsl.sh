#!/bin/bash
# Windows 폴더의 web/backend 를 WSL 의 ~/serving-robot-system/web/backend 로 복사하고 비교 (data/ 와 캐시는 제외)
S=/mnt/c/Users/user/Desktop/serving-robot-system/web/backend
D=$HOME/serving-robot-system/web/backend
mkdir -p "$D"
( cd "$S" && tar --exclude=data --exclude=fleet.yaml --exclude=__pycache__ --exclude=.pytest_cache -cf - . ) | ( cd "$D" && tar -xf - )
if diff -rq -x data -x fleet.yaml -x __pycache__ -x .pytest_cache "$S" "$D"; then echo "same: web/backend"; else echo "DIFF 있음"; fi
# 지도 설정 '적용 저장' 은 저장소의 tools/site_builder 를 불러 써요 (WSL clone 에서 git pull 로 받음)
[ -f "$HOME/serving-robot-system/tools/site_builder/site_builder/pipeline.py" ] || echo "주의: ~/serving-robot-system/tools/site_builder 가 없어요 (git pull 필요). 없으면 적용 저장이 503 SITE_BUILDER_MISSING 이에요."
