"""Names of the topics and services shared with other parts (no ROS).

Reads the shared file  config/common/interfaces.yaml  and answers
"what is the name of X for robot Y":

    names = names_for('robot_02')
    names.topic('person_limit')       # -> '/robot_02/person_limit'
    names.service('request_detour')   # -> '/robot_02/request_detour'
    names.nav2('nav_action')          # -> 'navigate_to_pose'
    names_for('')                     # '' = the default robot of the file

Nodes read their default names from here, so the names are written in one
place. A node parameter can still override a single name.

The file is searched like the site configuration (config_dir argument,
SERVING_ROBOT_CONFIG_DIR, installed package share, repository config/common).
"""
import os
import re

import yaml

from serving_robot_navigation.site_config import default_config_dir

FILE = 'interfaces.yaml'
_ROBOT = re.compile(r'^[a-z][a-z0-9_]*$')


class InterfaceError(ValueError):
    """interfaces.yaml is missing, malformed, or a name is unknown."""


class Names:
    def __init__(self, raw, robot):
        self._raw = raw
        self.robot = robot

    def _fill(self, text):
        return text.replace('{robot}', self.robot)

    def _lookup(self, section, key):
        table = self._raw.get(section)
        if not isinstance(table, dict) or key not in table:
            raise InterfaceError('%s has no %s "%s"' % (FILE, section, key))
        return self._fill(str(table[key]))

    def topic(self, key):
        return self._lookup('topics', key)

    def service(self, key):
        return self._lookup('services', key)

    def nav2(self, key):
        return self._lookup('nav2', key)

    def all_topics(self):
        return {k: self._fill(str(v)) for k, v in self._raw['topics'].items()}

    def all_services(self):
        return {k: self._fill(str(v)) for k, v in self._raw['services'].items()}


def load_raw(config_dir=None):
    path = os.path.join(config_dir or default_config_dir(), FILE)
    if not os.path.isfile(path):
        raise InterfaceError('missing config file: %s' % path)
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise InterfaceError('%s must be a mapping' % FILE)
    for section in ('topics', 'services', 'nav2'):
        if not isinstance(raw.get(section), dict):
            raise InterfaceError('%s has no "%s" section' % (FILE, section))
    if not _ROBOT.match(str(raw.get('default_robot', ''))):
        raise InterfaceError('%s: default_robot must look like robot_01' % FILE)
    return raw


def names_for(robot='', config_dir=None):
    """Names for `robot` ('' or None = the default robot of the file)."""
    raw = load_raw(config_dir)
    robot = str(robot or raw['default_robot'])
    if not _ROBOT.match(robot):
        raise InterfaceError('bad robot name %r (like robot_01)' % robot)
    return Names(raw, robot)
