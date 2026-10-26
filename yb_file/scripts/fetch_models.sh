#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/_local_only/models"
if [[ ! -s "$ROOT/_local_only/models/yolo11n.pt" ]]; then
  curl --fail --location https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt -o "$ROOT/_local_only/models/yolo11n.pt.part"
  mv "$ROOT/_local_only/models/yolo11n.pt.part" "$ROOT/_local_only/models/yolo11n.pt"
fi
set +u
source /opt/ros/jazzy/setup.bash
gz fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person'
