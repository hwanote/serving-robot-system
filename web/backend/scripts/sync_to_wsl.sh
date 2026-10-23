#!/bin/bash
# Windows 폴더의 web/backend 를 WSL 의 ~/serving-robot-system/web/backend 로 복사하고 비교 (data/ 와 캐시는 제외)
S=/mnt/c/Users/user/Desktop/serving-robot-system/web/backend
D=$HOME/serving-robot-system/web/backend
mkdir -p "$D"
( cd "$S" && tar --exclude=data --exclude=fleet.yaml --exclude=__pycache__ --exclude=.pytest_cache -cf - . ) | ( cd "$D" && tar -xf - )
if diff -rq -x data -x fleet.yaml -x __pycache__ -x .pytest_cache "$S" "$D"; then echo "same: web/backend"; else echo "DIFF 있음"; fi
