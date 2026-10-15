import os
from glob import glob

from setuptools import find_packages, setup


package_name = "serving_robot_mapping"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hwanote",
    maintainer_email="notihwan08@gmail.com",
    description="Autonomous mapping workflow for serving robots",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "exploration_manager = "
                "serving_robot_mapping.exploration_manager:main"
            ),
            "frontier_visualizer = serving_robot_mapping.frontier_visualizer:main",
            "frontier_goal_visualizer = serving_robot_mapping.frontier_goal_visualizer:main",
            "frontier_path_checker = serving_robot_mapping.frontier_path_checker:main",
        ],
    },
)
