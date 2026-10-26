#!/usr/bin/env bash
# VM side: copy files from the read-only shared folder into ~/serving-robot-system,
# convert CRLF to LF for text files, then verify every file against the manifest.
#   bash /mnt/hgfs/serving-robot-system/scripts/dev/vm_sync.sh
set -u
SRC="${SRC:-/mnt/hgfs/serving-robot-system}"
DST="${DST:-$HOME/serving-robot-system}"
PATHS="ros_ws/src config maps scripts tools/site_builder"
MAN="$SRC/scripts/dev/manifest.sha256"

[ -d "$SRC" ] || { echo "공유 폴더가 보이지 않아요: $SRC"; exit 1; }
[ -f "$MAN" ] || { echo "manifest가 없어요 (PC에서 make_manifest.sh를 먼저 실행해야 해요)"; exit 1; }
[ -d "$DST/.git" ] || { echo "VM 저장소가 없어요: $DST"; exit 1; }
command -v rsync >/dev/null 2>&1 || sudo apt-get install -y rsync

new=0
while read -r h t f; do [ -e "$DST/$f" ] || new=$((new+1)); done < "$MAN"

for p in $PATHS; do
  mkdir -p "$DST/$p"
  extra=""
  [ "$p" = "tools/site_builder" ] && extra="--exclude /out/"   # VM-generated results, never overwrite
  rsync -rl --no-perms --omit-dir-times $extra \
    --exclude '__pycache__' --exclude '*.pyc' --exclude '.git' \
    "$SRC/$p/" "$DST/$p/"
done

# CRLF -> LF for text files listed in the manifest
while read -r h t f; do
  [ "$t" = T ] && [ -f "$DST/$f" ] && sed -i 's/\r$//' "$DST/$f"
done < "$MAN"
# scripts and python entry points must be executable
find "$DST/scripts" "$DST/ros_ws/src" -type f \( -name '*.sh' -o -path '*/scripts/*.py' \) -exec chmod +x {} + 2>/dev/null

ok=0; bad=0; miss=0
while read -r h t f; do
  if [ ! -f "$DST/$f" ]; then miss=$((miss+1)); echo "없음: $f"; continue; fi
  if [ "$t" = T ]; then cur=$(tr -d '\r' < "$DST/$f" | sha256sum | cut -d' ' -f1)
  else cur=$(sha256sum "$DST/$f" | cut -d' ' -f1); fi
  if [ "$cur" = "$h" ]; then ok=$((ok+1)); else bad=$((bad+1)); echo "다름: $f"; fi
done < "$MAN"

echo "----"
echo "검증 결과: 일치 $ok / 다름 $bad / 없음 $miss  (새 파일 $new개)"
if [ "$bad" -eq 0 ] && [ "$miss" -eq 0 ]; then
  echo "PC와 VM 내용이 모두 같아요."
  [ "$new" -gt 0 ] && echo "새 파일이 있으니 빌드가 필요해요:  cd ~/serving-robot-system/ros_ws && colcon build --symlink-install && source install/setup.bash"
else
  echo "일치하지 않는 파일이 있어요. 위 목록을 알려 주세요."
fi
