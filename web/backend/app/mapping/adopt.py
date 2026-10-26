"""Make a saved mapping result the store's map (maps/stores/<store>/).

  saved navigation_map.pgm/yaml  ->  map.pgm, map.yaml, map_meta.yaml, map_preview.png

Steps (all under one lock; nothing is deleted):
  1. refuse while mapping runs, while orders are active, or when nothing was saved
  2. back up the current map folder and the editor's saved Semantic Map draft to
     <data_dir>/map_backups/<store>/<YYYYMMDD-HHMMSS>/
  3. install the new files (each replaced atomically), origin moved by `shift`
     (simulation: the SLAM frame starts at the robot's spawn pose; real store: 0, 0)
  4. Site reads the image again (revision + 1), the editor starts empty on the new map,
     the mapping draft is archived and the session goes back to idle.
A failure after step 3 puts the backed-up files back.

The robots read the map image only when they start, so the answer says they must be restarted.
config/common (nodes, tables) is left alone: it is regenerated from the editor later.
"""
import os
import shutil
import threading
import time

import yaml
from PIL import Image

from ..semantics import SemanticError, _atomic_json, _read_json
from ..util import now_iso


class MapAdopter:
    def __init__(self, site, fleet, semantics, session, data_dir, shift=(0.0, 0.0)):
        self.site, self.fleet, self.semantics = site, fleet, semantics
        self.session, self.data_dir = session, data_dir
        self.shift = (float(shift[0]), float(shift[1]))
        self.lock = threading.Lock()

    @property
    def store_dir(self):
        return os.path.join(self.site.repo, 'maps', 'stores', self.site.store_id)

    @property
    def backup_root(self):
        return os.path.join(self.data_dir, 'map_backups', self.site.store_id)

    def list_backups(self):
        out = []
        try:
            names = sorted(os.listdir(self.backup_root), reverse=True)
        except OSError:
            return out
        for name in names:
            meta = _read_json(os.path.join(self.backup_root, name, 'meta.json'))
            if isinstance(meta, dict):
                out.append(meta)
        return out

    # ---- helpers
    def _backup(self):
        stamp = time.strftime('%Y%m%d-%H%M%S')
        bid, n = stamp, 1
        while os.path.exists(os.path.join(self.backup_root, bid)):
            n += 1
            bid = '%s-%d' % (stamp, n)
        folder = os.path.join(self.backup_root, bid)
        os.makedirs(folder)
        shutil.copytree(self.store_dir, os.path.join(folder, 'map'))
        draft = self.semantics.load_draft()
        if draft is not None:
            _atomic_json(os.path.join(folder, 'semantics.json'), {'semantics': draft})
        meta = {'id': bid, 'created_at': now_iso(), 'kind': 'map_adoption',
                'note': '새 지도를 채택하기 전 지도', 'has_semantics': draft is not None,
                'map_revision': self.site.revision}
        _atomic_json(os.path.join(folder, 'meta.json'), meta)
        return meta

    def _put(self, src, name):
        tmp = os.path.join(self.store_dir, name + '.new')
        shutil.copyfile(src, tmp)
        os.replace(tmp, os.path.join(self.store_dir, name))

    def _write_text(self, name, text):
        tmp = os.path.join(self.store_dir, name + '.new')
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(text)
        os.replace(tmp, os.path.join(self.store_dir, name))

    def _restore(self, backup_id):
        src = os.path.join(self.backup_root, backup_id, 'map')
        for name in os.listdir(src):
            tmp = os.path.join(self.store_dir, name + '.new')
            shutil.copyfile(os.path.join(src, name), tmp)
            os.replace(tmp, os.path.join(self.store_dir, name))

    # ---- adopt
    def adopt(self):
        s = self.session
        with self.lock:
            if s.status()['phase'] == 'mapping':
                raise SemanticError(409, 'ALREADY_MAPPING', '매핑이 끝난 뒤에 지도를 반영할 수 있어요.')
            if not s.saved or not s.output_dir or not os.path.isdir(s.output_dir):
                raise SemanticError(409, 'NOT_SAVED', '저장한 보정 지도가 없어요. 먼저 지도를 보정해서 저장해 주세요.')
            if self.fleet.is_busy():
                raise SemanticError(409, 'ORDERS_ACTIVE',
                                    '주문이 진행 중이거나 대기 중이라 지도를 바꿀 수 없어요. 주문이 끝난 뒤에 다시 시도해 주세요.')
            src_yaml = os.path.join(s.output_dir, 'navigation_map.yaml')
            src_pgm = os.path.join(s.output_dir, 'navigation_map.pgm')
            try:
                with open(src_yaml, encoding='utf-8') as f:
                    meta = yaml.safe_load(f)
                with Image.open(src_pgm) as im:
                    width, height = im.size
                ox, oy, yaw = [float(v) for v in meta['origin']]
                res = float(meta['resolution'])
            except (OSError, ValueError, KeyError, TypeError) as e:
                raise SemanticError(422, 'SAVED_MAP_INVALID', '저장한 지도를 읽지 못했어요: %s' % e)

            old_meta = _read_json_yaml(os.path.join(self.store_dir, 'map_meta.yaml'))
            revision = int(old_meta.get('map_revision', 1)) + 1
            origin = [ox + self.shift[0], oy + self.shift[1], yaw]
            backup = self._backup()
            try:
                self._put(src_pgm, 'map.pgm')
                self._write_text('map.yaml', yaml.safe_dump({
                    'image': 'map.pgm', 'mode': 'trinary', 'resolution': res, 'origin': origin,
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}, sort_keys=False))
                self._write_text('map_meta.yaml', yaml.safe_dump({
                    'schema_version': 1, 'map_revision': revision, 'frame_id': 'map',
                    'source': 'slam_mapping', 'image': 'map.pgm', 'resolution_m': res,
                    'size_px': {'width': width, 'height': height},
                    'origin': {'x_m': origin[0], 'y_m': origin[1], 'yaw_rad': yaw},
                    'shift_origin_m': list(self.shift), 'adopted_at': now_iso(),
                    'backup_id': backup['id'],
                    'pixel_values': {'occupied': 0, 'free': 254, 'unknown': 205}}, sort_keys=False))
                preview = os.path.join(self.store_dir, 'map_preview.png.new')
                with Image.open(os.path.join(self.store_dir, 'map.pgm')) as im:
                    im.convert('L').save(preview, format='PNG')
                os.replace(preview, os.path.join(self.store_dir, 'map_preview.png'))
                self.site.reload_map_image()
            except Exception as e:
                try:
                    self._restore(backup['id'])
                    self.site.reload_map_image()
                except Exception:
                    pass
                raise SemanticError(500, 'ADOPT_FAILED', '지도를 바꾸다가 실패해서 원래대로 되돌렸어요: %s' % e)

            self._reset_editor()
            archived = s.finish_adoption()
            return {'adopted': True, 'backup': backup, 'map_revision': self.site.revision,
                    'map_meta_revision': revision, 'origin': origin, 'shift': list(self.shift),
                    'draft_archived_to': archived, 'needs_robot_restart': True,
                    'message': '새 지도를 적용했어요. 로봇을 재시작해 주세요. (새로운 지도가 적용되었습니다.)'}

    def _reset_editor(self):
        """The editor starts empty on the new map (the old draft is in the backup)."""
        empty = {'store_id': self.site.store_id, 'map_revision': self.site.revision,
                 'entities': [], 'waypoints': [], 'zones': []}
        try:
            self.semantics.save_draft(empty)
        except SemanticError:
            try:
                os.remove(self.semantics._draft_path)
            except OSError:
                pass


def _read_json_yaml(path):
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}
