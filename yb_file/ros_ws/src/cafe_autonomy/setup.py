from setuptools import setup, find_packages
from glob import glob
from pathlib import Path

package_name = 'cafe_autonomy'
share = 'share/' + package_name
files = [('share/ament_index/resource_index/packages', ['resource/' + package_name]),
         (share, ['package.xml'])]
for group in ('launch', 'config', 'models', 'worlds', 'maps', 'web'):
    contents = [str(p) for p in Path(group).rglob('*') if p.is_file()]
    if contents:
        
        for directory in sorted(set(str(p.parent) for p in map(Path,contents))):
            files.append((share + '/' + directory, [n for n in contents if str(Path(n).parent)==directory]))
setup(name=package_name, version='1.0.0', packages=find_packages(),
      data_files=files, install_requires=['setuptools', 'PyYAML'],
      zip_safe=True, maintainer='Cafe Autonomy Team', maintainer_email='team@example.invalid',
      description='Two-robot cafe autonomy integration: Gazebo, Nav2, fleet, serving and safety',
      license='Apache-2.0', entry_points={'console_scripts': [
          'fleet = cafe_autonomy.fleet:main',
          'mission = cafe_autonomy.mission:main',
          'semantic_safety = cafe_autonomy.safety:main',
          'odom_tf = cafe_autonomy.odom_tf:main',
          'map_odom_guard = cafe_autonomy.map_odom_guard:main',
          'scan_reframe = cafe_autonomy.scan_reframe:main',
          'metrics = cafe_autonomy.metrics:main',
          'rgbd_people = cafe_autonomy.rgbd_people:main',
          'zone_masks = cafe_autonomy.zone_masks:main',
          'lifecycle_guard = cafe_autonomy.lifecycle_guard:main',
          'zone_speed = cafe_autonomy.zone_speed:main',
          'dynamic_obstacles = cafe_autonomy.dynamic_obstacles:main',
'contact_loader = cafe_autonomy.contact_loader:main',
      ]})
