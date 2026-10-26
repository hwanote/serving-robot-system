#!/usr/bin/env python3
"""Compare Gazebo truth, odom and AMCL poses of robot_01 (run in the VM)."""
import math
import re
import subprocess

import yaml

SPAWN = (-3.2, -2.4)  # spawn_robot.launch.py x, y (yaw 0)


def run(cmd, t=25):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=t).stdout
    except subprocess.TimeoutExpired:
        return ""


def yaw_deg(z, w):
    return math.degrees(2 * math.atan2(z, w))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def truth():
    s = run(["gz", "topic", "-e", "-n", "1", "-t", "/world/cafe/pose/info"])
    m = re.search(
        r'name: "robot_01"\s+id: \d+\s+position \{\s+x: (\S+)\s+y: (\S+)\s+z: \S+\s+\}'
        r'\s+orientation \{\s+x: \S+\s+y: \S+\s+z: (\S+)\s+w: (\S+)', s)
    if not m:
        return None
    x, y, z, w = map(float, m.groups())
    return x, y, wrap(yaw_deg(z, w))


def echo(topic, off=(0.0, 0.0)):
    s = run(["ros2", "topic", "echo", "--once", topic])
    if not s.strip():
        return None
    p = yaml.safe_load(s.split("---")[0])["pose"]["pose"]
    q = p["orientation"]
    return (p["position"]["x"] + off[0], p["position"]["y"] + off[1],
            wrap(yaw_deg(q["z"], q["w"])))


def show(name, p):
    print(f"{name:<8}: " + ("(no data)" if p is None else
          f"x={p[0]:7.3f} y={p[1]:7.3f} yaw={p[2]:7.1f}"))


if __name__ == "__main__":
    t, o, a = truth(), echo("/robot_01/odom", SPAWN), echo("/amcl_pose")
    show("truth", t)
    show("odom", o)
    show("amcl", a)
    if t and o:
        print(f"odom-truth: dist={math.hypot(o[0]-t[0], o[1]-t[1]):.3f} m  dyaw={wrap(o[2]-t[2]):.1f} deg")
    if t and a:
        print(f"amcl-truth: dist={math.hypot(a[0]-t[0], a[1]-t[1]):.3f} m  dyaw={wrap(a[2]-t[2]):.1f} deg")
