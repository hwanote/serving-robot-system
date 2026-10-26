#!/usr/bin/env python3
"""Check that every section module is plugged in everywhere it must be.

    python3 scripts/validate_layout.py

Sections: monitoring, decision, motion, command (ros_ws/src/serving_robot_navigation).
For each section it checks:
  - the package folder has __init__.py and a README.md
  - every pure module (<package>/<section>/*.py) is named in the README
  - every node (scripts/<section>/*.py) is installed by CMakeLists.txt, named in
    the README and, unless it is a command line tool (CLI_TOOLS), started by
    launch/<section>.launch.py
  - every `*.py` executable of a section launch file is a node of that section
  - every `scripts/<section>/...` line of CMakeLists.txt points to a real file
  - nothing is left outside the sections (scripts/*.py, extra package modules)
  - navigation.launch.py includes every section launch file
Exit code 0 = all plugged in, 1 = problems printed.
"""
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SECTIONS = ('monitoring', 'decision', 'motion', 'command')
CLI_TOOLS = {'go_to.py', 'serve.py', 'decide.py', 'reset.py'}            # run by hand, not by a launch file
SHARED = {'__init__.py', 'site_config.py', 'interfaces.py', 'launch_params.py'}


def _read(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


def _py(folder):
    if not os.path.isdir(folder):
        return []
    return sorted(f for f in os.listdir(folder) if f.endswith('.py'))


def check(root=ROOT):
    nav = os.path.join(root, 'ros_ws', 'src', 'serving_robot_navigation')
    pkg = os.path.join(nav, 'serving_robot_navigation')
    problems = []
    cmake = _read(os.path.join(nav, 'CMakeLists.txt'))
    nav_launch = os.path.join(nav, 'launch', 'navigation.launch.py')
    nav_launch_text = _read(nav_launch) if os.path.isfile(nav_launch) else ''

    for section in SECTIONS:
        folder = os.path.join(pkg, section)
        readme_path = os.path.join(folder, 'README.md')
        launch_path = os.path.join(nav, 'launch', '%s.launch.py' % section)
        if not os.path.isfile(os.path.join(folder, '__init__.py')):
            problems.append('%s: package folder has no __init__.py' % section)
        if not os.path.isfile(readme_path):
            problems.append('%s: README.md is missing' % section)
            readme = ''
        else:
            readme = _read(readme_path)
        for mod in _py(folder):
            if mod != '__init__.py' and mod not in readme:
                problems.append('%s: module %s is not named in the README'
                                % (section, mod))

        nodes = _py(os.path.join(nav, 'scripts', section))
        launch = _read(launch_path) if os.path.isfile(launch_path) else None
        for node in nodes:
            if 'scripts/%s/%s' % (section, node) not in cmake:
                problems.append('%s: node %s is not installed in CMakeLists.txt'
                                % (section, node))
            if node not in readme:
                problems.append('%s: node %s is not named in the README'
                                % (section, node))
            if node in CLI_TOOLS:
                continue
            if launch is None:
                problems.append('%s: node %s has no launch/%s.launch.py'
                                % (section, node, section))
            elif "'%s'" % node not in launch:
                problems.append('%s: node %s is not started by %s.launch.py'
                                % (section, node, section))
        if launch is not None:
            for exe in sorted(set(re.findall(r"executable='([^']+\.py)'", launch))):
                if exe not in nodes:
                    problems.append('%s.launch.py starts %s, which is not in '
                                    'scripts/%s/' % (section, exe, section))
            if nav_launch_text and '%s.launch.py' % section not in nav_launch_text:
                problems.append('navigation.launch.py does not include %s.launch.py'
                                % section)

    for line in re.findall(r'scripts/[a-z_]+/[A-Za-z0-9_]+\.py', cmake):
        if not os.path.isfile(os.path.join(nav, line)):
            problems.append('CMakeLists.txt installs %s, which does not exist' % line)
    for stray in _py(os.path.join(nav, 'scripts')):
        problems.append('scripts/%s is outside the sections' % stray)
    for mod in _py(pkg):
        if mod not in SHARED:
            problems.append('serving_robot_navigation/%s is outside the sections '
                            '(shared files: %s)' % (mod, ', '.join(sorted(SHARED))))
    return problems


def main():
    problems = check()
    for p in problems:
        print('PROBLEM', p)
    print('layout: %s' % ('all plugged in' if not problems
                          else '%d problem(s)' % len(problems)))
    return 1 if problems else 0


if __name__ == '__main__':
    raise SystemExit(main())
