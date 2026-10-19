#!/usr/bin/env bash
# PC side: write scripts/dev/manifest.sha256 (hash of every synced file).
# Text files are hashed with CR removed, binary files are hashed as-is.
# Columns: <sha256>  <T|B>  <path>
set -eu
cd "$(dirname "$0")/../.."
PATHS="ros_ws/src config maps scripts tools/site_builder"
OUT=scripts/dev/manifest.sha256
: > "$OUT.tmp"
find $PATHS -type f \
  -not -path 'tools/site_builder/out/*' -not -path '*/__pycache__/*' -not -name '*.pyc' -not -name 'manifest.sha256*' \
  | LC_ALL=C sort | while read -r f; do
  case "$f" in
    *.py|*.yaml|*.yml|*.xml|*.xacro|*.rviz|*.sdf|*.world|*.urdf|*.txt|*.md|*.cfg|*.msg|*.srv|*.json|*.sh|*.cmake|*.xml.in|*.gitkeep)
      t=T; h=$(tr -d '\r' < "$f" | sha256sum | cut -d' ' -f1);;
    *)
      t=B; h=$(sha256sum "$f" | cut -d' ' -f1);;
  esac
  printf '%s  %s  %s\n' "$h" "$t" "$f" >> "$OUT.tmp"
done
mv "$OUT.tmp" "$OUT"
echo "manifest: $(wc -l < "$OUT") files -> $OUT"
