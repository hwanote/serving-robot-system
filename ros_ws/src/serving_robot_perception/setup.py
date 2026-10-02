import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'serving_robot_perception'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='hwanote',
    maintainer_email='notihwan08@gmail.com',

    description=(
        'Perception and semantic safety nodes '
        'for the serving robot system.'
    ),

    license='TODO',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
        ],
    },
)
