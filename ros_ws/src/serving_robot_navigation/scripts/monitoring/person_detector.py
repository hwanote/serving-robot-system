#!/usr/bin/env python3
"""Person detector (camera node): YOLO on the RGB images + the depth images -> what it saw.

    ros2 run serving_robot_navigation person_detector.py --ros-args -p use_sim_time:=true

    in   per camera of `cameras` (names from config/common/interfaces.yaml):
             /robot_01/<camera>/image        sensor_msgs/Image (rgb8)
             /robot_01/<camera>/depth        sensor_msgs/Image (32FC1, metres)
             /robot_01/<camera>/camera_info  sensor_msgs/CameraInfo (fx, cx)
         TF base_footprint -> <camera>_link (which way the camera looks)
    out  /robot_01/person_observations  std_msgs/String, JSON, one per camera per
             tick (rate_hz): {camera, time, new_frame, count,
                              people: [{distance_m, bearing_rad, confidence, bbox}]}
         (see monitoring/person_observation.py)

Monitoring only: it reports the people it sees, their real distance (the depth
along the optical axis divided by the cosine of the pixel angle) and their
direction seen from the robot. The state (NORMAL / SLOW / STOP) is decided by
person_state (scripts/decision/), which publishes /robot_01/person_detection.

One node, several cameras, one YOLO model (a GPU-less VM cannot afford three):
every tick it runs the primary camera and the next of the other cameras in
turn, in one batch. The observation of every camera is sent every tick, also
without a new image, so the judge knows the camera node is alive; the primary
camera goes last. For one node per camera give each a `cameras` list of one
and a distinct node name.

YOLO runs from a separate virtual environment (default ~/yolo_venv): its
packages are appended to sys.path, so the ROS Python packages (numpy, ...) keep
priority. Images are decoded here without cv_bridge.
"""
import glob
import json
import math
import os
import sys

import numpy as np  # noqa: E402  (before the venv path is added: the system numpy wins)
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from serving_robot_navigation.interfaces import names_for
from serving_robot_navigation.monitoring.person_observation import (
    box_distance, fx_from_fov, make_observation, pixel_angle, true_range, wrap)


def camera_topics(names, cam):
    """image, depth, camera_info topics of a camera (interfaces.yaml)."""
    if cam == 'camera_front':       # the first camera kept its original keys
        keys = ('camera_image', 'camera_depth', 'camera_info')
    else:
        keys = (cam + '_image', cam + '_depth', cam + '_info')
    return [names.topic(k) for k in keys]


def add_venv(pattern):
    for path in glob.glob(os.path.expanduser(pattern)):
        if path not in sys.path:
            sys.path.append(path)


def rgb_array(msg):
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)[:, :w * 3]
    arr = arr.reshape(h, w, 3)
    return arr[:, :, ::-1] if msg.encoding == 'bgr8' else arr


def depth_array(msg):
    h, w = msg.height, msg.width
    if msg.encoding == '16UC1':
        arr = np.frombuffer(msg.data, np.uint16).reshape(h, msg.step // 2)[:, :w]
        return arr.astype(np.float32) / 1000.0
    return np.frombuffer(msg.data, np.float32).reshape(h, msg.step // 4)[:, :w]


class Camera:
    """What the node keeps per camera."""

    def __init__(self, name, yaw_fallback):
        self.name = name
        self.rgb = None
        self.rgb_seq = 0
        self.done_seq = 0
        self.depth = None
        self.depth_t = None
        self.fx = None
        self.cx = None
        self.yaw = None              # from TF (base frame -> camera link)
        self.yaw_fallback = yaw_fallback
        self.last_debug = 0.0


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        d = self.declare_parameter
        nm = names_for(d('robot_name', '').value)   # '' = default_robot
        d('cameras', ['camera_front', 'camera_rear_left', 'camera_rear_right'])
        d('primary_camera', 'camera_front')         # run every tick
        d('camera_yaws_deg', [0.0, 120.0, -120.0])  # only if TF has no camera link
        d('hfov_rad', 2.094)                        # only before camera_info arrives
        d('base_frame', 'base_footprint')
        d('out_topic', nm.topic('person_observations'))
        d('model_path', '~/yolo_models/yolov8n.pt')
        d('venv_site_packages', '~/yolo_venv/lib/python3*/site-packages')
        d('confidence', 0.4)
        d('imgsz', 320)
        d('rate_hz', 4.0)
        d('min_depth_m', 0.1)
        d('max_depth_m', 8.0)
        d('max_depth_age_s', 0.5)
        d('debug_dir', '')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g

        add_venv(g('venv_site_packages'))
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit('ultralytics is not installed (%s). Install it in '
                             '~/yolo_venv (see docs/RUN_GUIDE.md)' % e)
        self.get_logger().info('loading YOLO model %s ...' % g('model_path'))
        self.model = YOLO(os.path.expanduser(g('model_path')))
        # The first inference is slow (set-up): do it now, before the timer,
        # so the first real tick does not leave a gap (a false DETECTOR_SILENT
        # right after DETECTOR_RECOVERED). Batch of 2 = the size used per tick.
        blank = np.zeros((240, 320, 3), dtype=np.uint8)
        self.model.predict([blank, blank], imgsz=int(g('imgsz')),
                           conf=float(g('confidence')), classes=[0],
                           device='cpu', verbose=False)
        self.get_logger().info('YOLO warmed up')

        names = list(g('cameras'))
        yaws = list(g('camera_yaws_deg'))
        self.primary = g('primary_camera') if g('primary_camera') in names else names[0]
        self.cams = {}
        for k, name in enumerate(names):
            cam = Camera(name, math.radians(yaws[k]) if k < len(yaws) else 0.0)
            self.cams[name] = cam
            image, depth, info = camera_topics(nm, name)
            self.create_subscription(Image, image, lambda m, c=cam: self._on_rgb(c, m), 2)
            self.create_subscription(Image, depth, lambda m, c=cam: self._on_depth(c, m), 2)
            self.create_subscription(CameraInfo, info,
                                     lambda m, c=cam: self._on_info(c, m), 2)
        self.others = [n for n in names if n != self.primary]
        self.turn = 0
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(String, g('out_topic'), 10)
        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.get_logger().info('person detector ready: %s (primary %s) -> %s, %.1f Hz' % (
            ', '.join(names), self.primary, g('out_topic'), float(g('rate_hz'))))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_rgb(self, cam, msg):
        cam.rgb = msg
        cam.rgb_seq += 1

    def _on_depth(self, cam, msg):
        cam.depth = msg
        cam.depth_t = self._now()

    def _on_info(self, cam, msg):
        if msg.k[0] > 0.0:
            cam.fx, cam.cx = float(msg.k[0]), float(msg.k[2])

    def _yaw(self, cam):
        if cam.yaw is None:
            try:
                t = self.tf.lookup_transform(self.g('base_frame'), cam.name + '_link',
                                             Time())
                q = t.transform.rotation
                cam.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.get_logger().info('%s looks at %.0f deg (TF)'
                                       % (cam.name, math.degrees(cam.yaw)))
            except TransformException:
                return cam.yaw_fallback
        return cam.yaw

    def _people(self, cam, img_w, res, now):
        """YOLO boxes -> (distance, confidence, box, bearing) with the real distance."""
        depth_ok = (cam.depth is not None and cam.depth_t is not None
                    and now - cam.depth_t <= float(self.g('max_depth_age_s')))
        depth = depth_array(cam.depth) if depth_ok else None
        fx = cam.fx or fx_from_fov(img_w, float(self.g('hfov_rad')))
        cx = cam.cx if cam.cx is not None else img_w / 2.0
        yaw = self._yaw(cam)
        found = []
        for b in res.boxes:
            box = [float(v) for v in b.xyxy[0].tolist()]
            off = pixel_angle((box[0] + box[2]) / 2.0, cx, fx)
            dist = None
            if depth is not None:
                z = box_distance(depth, box, float(self.g('min_depth_m')),
                                 float(self.g('max_depth_m')))
                dist = None if z is None else true_range(z, off)
            found.append((dist, float(b.conf[0]), box, wrap(yaw + off)))
        return found

    def _tick(self):
        now = self._now()
        run = [self.primary]
        if self.others:
            run.append(self.others[self.turn % len(self.others)])
            self.turn += 1
        batch = []
        for name in run:
            cam = self.cams[name]
            if cam.rgb is not None and cam.rgb_seq != cam.done_seq:
                cam.done_seq = cam.rgb_seq
                img = np.ascontiguousarray(rgb_array(cam.rgb)[:, :, ::-1])  # BGR
                batch.append((cam, img))
        found = {}
        if batch:
            results = self.model.predict([img for _c, img in batch],
                                         imgsz=int(self.g('imgsz')),
                                         conf=float(self.g('confidence')), classes=[0],
                                         device='cpu', verbose=False)
            for (cam, img), res in zip(batch, results):
                found[cam.name] = self._people(cam, img.shape[1], res, now)
                self._debug(cam, img, found[cam.name], now)
        for name in self.others + [self.primary]:     # primary last: judge sees fresh sides
            obs = make_observation(name, now, name in found, found.get(name, []))
            self.pub.publish(String(data=json.dumps(obs)))

    def _debug(self, cam, img, found, now):
        folder = self.g('debug_dir')
        if not folder or now - cam.last_debug < 1.0:
            return
        cam.last_debug = now
        try:
            import cv2
            out = img.copy()
            for dist, conf, box, _bearing in found:
                x1, y1, x2, y2 = [int(v) for v in box]
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = '%.2f %s' % (conf, '-' if dist is None else '%.2fm' % dist)
                cv2.putText(out, label, (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            os.makedirs(os.path.expanduser(folder), exist_ok=True)
            cv2.imwrite(os.path.join(os.path.expanduser(folder),
                                     'latest_%s.jpg' % cam.name), out)
        except Exception as e:  # debug output must never stop the detector
            self.get_logger().info('debug image failed: %s' % e)


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
