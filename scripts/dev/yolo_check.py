#!/usr/bin/env python3
"""Check that YOLO works in ~/yolo_venv (no ROS, no simulation).

    cd ~/yolo_models && ~/yolo_venv/bin/python ~/serving-robot-system/scripts/dev/yolo_check.py

Runs yolov8n on the sample picture that ships with ultralytics (a bus with
several people) and prints what it found. Expect 'person' 3-4 times.
"""
import os
import sys

import ultralytics
from ultralytics import YOLO

image = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(ultralytics.__file__), 'assets', 'bus.jpg')
model = YOLO(os.path.expanduser('~/yolo_models/yolov8n.pt'))
res = model.predict(image, imgsz=320, conf=0.4, device='cpu', verbose=False)[0]
names = [res.names[int(b.cls[0])] for b in res.boxes]
print('image:', image)
print('found:', names)
print('persons:', names.count('person'))
