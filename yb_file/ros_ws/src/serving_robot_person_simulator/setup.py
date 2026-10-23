from setuptools import setup
from glob import glob
import os

package_name = 'serving_robot_person_simulator'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Local developer',
    maintainer_email='maintainer@example.invalid',
    description='Standalone dynamic pedestrian scenarios for Gazebo Harmonic',
    license='Apache-2.0',
    entry_points={'console_scripts': ['manager = serving_robot_person_simulator.manager:main',
                                     'watchdog = serving_robot_person_simulator.watchdog:main']},
)
