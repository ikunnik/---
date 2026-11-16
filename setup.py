from pathlib import Path
from setuptools import find_packages, setup

PACKAGE_NAME = "rover_x1"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/rover_x1"]),
    ("share/rover_x1", ["package.xml"]),
    ("share/rover_x1/launch", ["launch/rover.launch.py"]),
    ("share/rover_x1/config", ["config/rover.yaml", "config/ros_camera.example.yaml"]),
    ("share/rover_x1/docs", ["docs/ros2.md", "docs/ros2_migration_plan.md",
                             "docs/ros2_interfaces.md", "docs/ros2_phase1.md", "docs/ros2_camera.md"]),
]
for directory in [Path("assets"), Path("assets/voice")]:
    assets = sorted(str(p) for p in directory.iterdir() if p.is_file())
    data_files.append((f"share/{PACKAGE_NAME}/{directory}", assets))

setup(
    name=PACKAGE_NAME,
    version="7.5.0",
    packages=find_packages(include=[
        "rover_x1", "rover_x1.*", "app", "app.*", "runtime", "runtime.*",
        "motion", "motion.*", "decisions", "decisions.*", "workers", "workers.*",
    ]),
    py_modules=["actions", "comm", "config", "messages", "robot_factory",
                "robot_link", "ros2_robot_link"],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    tests_require=["pytest"],
    maintainer="Rover X1 maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 packaging and unified launch for Rover X1",
    license="Proprietary",
    entry_points={"console_scripts": ["controller = rover_x1.controller:main",
                                      "camera_inventory = rover_x1.camera_inventory:main"]},
)
