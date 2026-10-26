"""Put an applied map into config/common (where the robots read it), with dated backups.

promote   data/generated/<store>/{entities,waypoints,zones}.yaml  ->  config/common/
restore   data/backups/<store>/<id>/...                           ->  config/common/

Every replacement first copies the current config/common files to a new backup folder
data/backups/<store>/<YYYYMMDD-HHMMSS>/ (nothing is deleted), so any older map can be
brought back, and a restore can itself be undone.

The robots read config/common only when their nodes start or when they get the
`reload_map` command (robot card button), so the response always says that they
have to read it again. The backend's own Site and the fleet are
refreshed here.
"""
import hashlib
import os
import shutil
import sys
import threading
import time

import yaml

from .site import semantics_from_config
from .semantics import SemanticError, _atomic_json, _read_json, digest, validate
from .util import now_iso

FILES = ('entities.yaml', 'waypoints.yaml', 'zones.yaml')


def _sha(path):
    h = hashlib.sha256()
    try:
        with open(path, 'rb') as f:
            h.update(f.read())
    except OSError:
        return None
    return h.hexdigest()


def _dir_sha(folder):
    h = hashlib.sha256()
    for name in FILES:
        s = _sha(os.path.join(folder, name))
        if s is None:
            return None
        h.update(name.encode() + s.encode())
    return h.hexdigest()


def _summary(folder):
    """Counts shown next to a backup so a person can tell the versions apart."""
    try:
        with open(os.path.join(folder, 'waypoints.yaml'), encoding='utf-8') as f:
            wp = yaml.safe_load(f) or {}
        with open(os.path.join(folder, 'entities.yaml'), encoding='utf-8') as f:
            ent = yaml.safe_load(f) or {}
    except (OSError, ValueError):
        return None
    return {'nodes': len(wp.get('nodes') or {}), 'edges': len(wp.get('edges') or []),
            'tables': len(ent.get('tables') or {})}


_REV_CACHE = {}


def live_revision(repo):
    """Short hash of config/common as the robots compute it (Site.revision), so it can be
    compared with the `config_revision` a robot reports. None when it cannot be read."""
    common = os.path.join(repo, 'config', 'common')
    try:
        key = tuple(os.stat(os.path.join(common, n)).st_mtime_ns for n in FILES)
    except OSError:
        return None
    hit = _REV_CACHE.get(common)
    if hit and hit[0] == key:
        return hit[1]
    nav = os.path.join(repo, 'ros_ws', 'src', 'serving_robot_navigation')
    if not os.path.isdir(nav):
        return None
    if nav not in sys.path:
        sys.path.insert(0, nav)
    try:
        from serving_robot_navigation.site_config import config_revision
        rev = config_revision(common)
    except Exception:
        return None
    _REV_CACHE[common] = (key, rev)
    return rev


_IMG_CACHE = {}


def live_map_image(repo, store_id):
    """Fingerprint of maps/stores/<store>/map.yaml + its image, as the robots compute it
    (mission_status.map_image) so the two can be compared. None when it cannot be read.
    A robot that started before the map picture changed reports an older value."""
    mdir = os.path.join(repo, 'maps', 'stores', store_id)
    yml = os.path.join(mdir, 'map.yaml')
    try:
        key = tuple((os.stat(p).st_mtime_ns, os.stat(p).st_size)
                    for p in (yml, os.path.join(mdir, 'map.pgm')))
    except OSError:
        return None
    hit = _IMG_CACHE.get(yml)
    if hit and hit[0] == key:
        return hit[1]
    nav = os.path.join(repo, 'ros_ws', 'src', 'serving_robot_navigation')
    if not os.path.isdir(nav):
        return None
    if nav not in sys.path:
        sys.path.insert(0, nav)
    try:
        from serving_robot_navigation.site_config import map_image_fingerprint
        value = map_image_fingerprint(yml) or None
    except Exception:
        return None
    _IMG_CACHE[yml] = (key, value)
    return value


class LiveConfig:
    def __init__(self, site, fleet, repo, data_dir):
        self.site, self.fleet, self.repo, self.data_dir = site, fleet, repo, data_dir
        self.lock = threading.Lock()

    # ---- paths
    @property
    def common(self):
        return os.path.join(self.repo, 'config', 'common')

    @property
    def _gen_dir(self):
        return os.path.join(self.data_dir, 'generated', self.site.store_id)

    @property
    def _backup_root(self):
        return os.path.join(self.data_dir, 'backups', self.site.store_id)

    @property
    def _live_path(self):
        return os.path.join(self.data_dir, 'live', '%s.json' % self.site.store_id)

    @property
    def _live_sem_path(self):
        # the saved draft that produced what is in config/common right now
        return os.path.join(self.data_dir, 'live', '%s.semantics.json' % self.site.store_id)

    @property
    def _draft_path(self):
        return os.path.join(self.data_dir, 'semantics', '%s.json' % self.site.store_id)

    # ---- what is live
    def revision(self):
        return live_revision(self.repo)

    def status(self):
        applied = _read_json(os.path.join(self._gen_dir, 'applied.json'))
        live_sha, gen_sha = _dir_sha(self.common), _dir_sha(self._gen_dir)
        return {'store_id': self.site.store_id, 'live_sha': live_sha,
                'matches_applied': bool(live_sha and live_sha == gen_sha),
                'applied_source_sha': (applied or {}).get('source_sha'),
                'last': _read_json(self._live_path), 'backup_count': len(self.list_backups()),
                'needs_robot_restart_note': '노드·통로·테이블 위치는 로봇 관제 화면의 "지도 다시 읽기"로 바뀌어요. 지도 그림(map.pgm)이 바뀐 경우에는 다시 읽기로는 안 되고 로봇을 재시작해야 해요.'}

    def list_backups(self):
        out = []
        try:
            names = sorted(os.listdir(self._backup_root), reverse=True)
        except OSError:
            return out
        for name in names:
            folder = os.path.join(self._backup_root, name)
            meta = _read_json(os.path.join(folder, 'meta.json'))
            if not isinstance(meta, dict) or _dir_sha(folder) is None:
                continue
            meta['summary'] = _summary(folder)
            meta['has_semantics'] = os.path.isfile(os.path.join(folder, 'semantics.json'))
            out.append(meta)
        return out

    # ---- guards
    def _check_idle(self):
        if self.fleet.is_busy():
            raise SemanticError(409, 'ORDERS_ACTIVE',
                                '주문이 진행 중이거나 대기 중이라 지도를 바꿀 수 없어요. '
                                '주문이 끝난 뒤에 다시 시도해 주세요.')

    def _check_loads(self, folder):
        """The robots' own loader (and the poses this backend needs) must accept the files."""
        for name in FILES:
            if not os.path.isfile(os.path.join(folder, name)):
                raise SemanticError(422, 'INCOMPLETE', '지도 파일이 빠져 있어요: %s' % name)
        nav = os.path.join(self.repo, 'ros_ws', 'src', 'serving_robot_navigation')
        if os.path.isdir(nav):
            if nav not in sys.path:
                sys.path.insert(0, nav)
            from serving_robot_navigation.site_config import SiteConfigError, load_site
            try:
                load_site(folder)
            except (SiteConfigError, yaml.YAMLError, OSError, KeyError, TypeError,
                    ValueError, AttributeError) as e:
                raise SemanticError(422, 'ROBOT_LOADER_REJECTED',
                                    '로봇이 읽을 수 없는 지도예요: %s' % e)
        with open(os.path.join(folder, 'waypoints.yaml'), encoding='utf-8') as f:
            poses = (yaml.safe_load(f) or {}).get('poses') or {}
        missing = [r for r in self.fleet.robots
                   if 'standby_%s' % r.rsplit('_', 1)[-1] not in poses]
        if missing:
            raise SemanticError(
                422, 'MISSING_STANDBY',
                '로봇마다 대기 위치(standby)가 필요한데 없는 로봇이 있어요: %s' % ', '.join(missing),
                {'robots': missing})
        if self.fleet.kitchen not in poses:
            raise SemanticError(422, 'MISSING_KITCHEN', '주방(%s) 위치가 없어요.' % self.fleet.kitchen)

    # ---- the swap
    def _semantics_for_backup(self, kind):
        """The editor's saved draft that goes with the map being backed up (None if unknown).
        promote: the draft that produced the current config/common (only if config/common is
        still exactly what it produced). before_restore: what the editor shows right now, so
        undoing a restore brings the editor back too."""
        if kind == 'before_restore':
            saved = _read_json(self._draft_path)
            doc = saved.get('semantics') if isinstance(saved, dict) else None
            return doc if isinstance(doc, dict) else None
        rec = _read_json(self._live_sem_path)
        if isinstance(rec, dict) and isinstance(rec.get('semantics'), dict) \
                and rec.get('common_sha') == _dir_sha(self.common):
            return rec['semantics']
        return None

    def _new_backup(self, kind, note, source_sha=None):
        stamp = time.strftime('%Y%m%d-%H%M%S')
        bid, n = stamp, 1
        while os.path.exists(os.path.join(self._backup_root, bid)):
            n += 1
            bid = '%s-%d' % (stamp, n)
        folder = os.path.join(self._backup_root, bid)
        os.makedirs(folder)
        for name in FILES:
            shutil.copy2(os.path.join(self.common, name), os.path.join(folder, name))
        doc = self._semantics_for_backup(kind)
        if doc is not None:
            _atomic_json(os.path.join(folder, 'semantics.json'), {'semantics': doc})
        meta = {'id': bid, 'created_at': now_iso(), 'kind': kind, 'note': note,
                'sha': _dir_sha(folder), 'source_sha': source_sha,
                'has_semantics': doc is not None}
        _atomic_json(os.path.join(folder, 'meta.json'), meta)
        return meta

    def _install(self, folder, backup_folder):
        done = []
        try:
            for name in FILES:
                tmp = os.path.join(self.common, name + '.new')
                shutil.copyfile(os.path.join(folder, name), tmp)
                os.replace(tmp, os.path.join(self.common, name))
                done.append(name)
            self.site.reload()
            self.fleet.refresh_site()
        except Exception as e:
            for name in FILES:
                try:
                    shutil.copyfile(os.path.join(backup_folder, name),
                                    os.path.join(self.common, name + '.new'))
                    os.replace(os.path.join(self.common, name + '.new'),
                               os.path.join(self.common, name))
                except OSError:
                    pass
            try:
                self.site.reload()
                self.fleet.refresh_site()
            except Exception:
                pass
            raise SemanticError(500, 'PROMOTE_FAILED',
                                '지도를 바꾸다가 실패해서 원래대로 되돌렸어요: %s' % e)

    def _record(self, kind, backup, source_sha, note):
        info = {'at': now_iso(), 'kind': kind, 'backup_id': backup['id'], 'source_sha': source_sha,
                'note': note, 'sha': _dir_sha(self.common), 'needs_robot_restart': True}
        _atomic_json(self._live_path, info)
        return info

    def promote(self):
        applied = _read_json(os.path.join(self._gen_dir, 'applied.json'))
        if not applied:
            raise SemanticError(409, 'NOT_APPLIED', '적용 결과가 없어요. 먼저 적용해 주세요.')
        draft = _read_json(os.path.join(self.data_dir, 'semantics', '%s.json' % self.site.store_id))
        if not draft or draft.get('sha') != applied.get('source_sha'):
            raise SemanticError(409, 'NOT_IN_SYNC',
                                '저장한 초안이 적용 결과보다 새로워요. 다시 적용한 뒤 반영해 주세요.')
        with self.lock:
            self._check_idle()
            self._check_loads(self._gen_dir)
            if _dir_sha(self._gen_dir) == _dir_sha(self.common):
                raise SemanticError(409, 'ALREADY_LIVE', '이미 로봇이 읽는 지도와 같아요.')
            backup = self._new_backup('promote', '적용 결과를 반영하기 전 지도',
                                      source_sha=applied.get('source_sha'))
            self._install(self._gen_dir, os.path.join(self._backup_root, backup['id']))
            _atomic_json(self._live_sem_path, {'common_sha': _dir_sha(self.common),
                                               'semantics': draft.get('semantics')})
            return {'backup': backup,
                    'live': self._record('promote', backup, applied.get('source_sha'),
                                         '적용 결과를 로봇 지도로 반영')}

    def restore(self, backup_id):
        folder = os.path.join(self._backup_root, os.path.basename(backup_id))
        if os.path.basename(backup_id) != backup_id or not os.path.isdir(folder) \
                or _dir_sha(folder) is None:
            raise SemanticError(404, 'BACKUP_NOT_FOUND', '그 백업을 찾지 못했어요: %s' % backup_id)
        with self.lock:
            self._check_idle()
            self._check_loads(folder)
            if _dir_sha(folder) == _dir_sha(self.common):
                raise SemanticError(409, 'ALREADY_LIVE', '이미 그 백업과 같은 지도예요.')
            before = self._new_backup('before_restore', '%s 백업으로 되돌리기 전 지도' % backup_id)
            self._install(folder, os.path.join(self._backup_root, before['id']))
            doc, note = self._restore_draft(folder)
            return {'backup': before,
                    'live': self._record('restore', before, None, '%s 백업으로 되돌림' % backup_id),
                    'semantics': doc, 'semantics_restored': doc is not None,
                    'semantics_note': note}

    def _restore_draft(self, folder):
        """Bring the editor's saved draft back too. Returns (doc or None, note for the person).
        A backup made before drafts were kept has none: the editor view is rebuilt from the
        backup's own map files instead (positions of tables, kitchen and robot waiting spots;
        zones the person drew are not in those files and come back empty)."""
        saved = _read_json(os.path.join(folder, 'semantics.json'))
        doc = saved.get('semantics') if isinstance(saved, dict) else None
        note = None
        if not isinstance(doc, dict):
            try:
                with open(os.path.join(folder, 'waypoints.yaml'), encoding='utf-8') as f:
                    wp = yaml.safe_load(f)
                with open(os.path.join(folder, 'entities.yaml'), encoding='utf-8') as f:
                    ent = yaml.safe_load(f) or {}
                doc = semantics_from_config(wp, ent, self.site.store_id, self.site.revision)
            except Exception as e:
                return None, '편집 화면을 다시 만들지 못해 로봇 지도만 되돌렸어요: %s' % e
            note = '이 백업에는 편집 화면 저장본이 없어서 지도 파일에서 다시 만들었어요. 직접 그린 구역은 빠져요.'
        try:
            validate(doc, self.site.store_id, self.site.revision)
        except SemanticError as e:
            return None, '편집 화면 저장본이 지금 지도와 맞지 않아 로봇 지도만 되돌렸어요: %s' % e.message
        _atomic_json(self._draft_path, {'saved_at': now_iso(), 'sha': digest(doc), 'semantics': doc})
        _atomic_json(self._live_sem_path, {'common_sha': _dir_sha(self.common), 'semantics': doc})
        return doc, note
