from glob import glob
from setuptools import find_packages, setup

setup(
    name="logistics_robot",
    version="0.1.0",
    packages=find_packages(),
    package_data={"logistics_robot": ["web/*.html", "web/*.css", "web/*.js"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/logistics_robot"]),
        ("share/logistics_robot", ["package.xml"]),
        ("share/logistics_robot/config", glob("config/*.yaml")),
        ("share/logistics_robot/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "qr_scanner = logistics_robot.qr_node:main",
        "safety_gate = logistics_robot.safety_node:main",
        "mission = logistics_robot.mission_node:main",
        "odom_tf = logistics_robot.odom_node:main",
        "teleop = logistics_robot.teleop:main",
        "lidar_ms200p = logistics_robot.lidar_node:main",
        "phone_remote = logistics_robot.web_node:main",
    ]},
)
