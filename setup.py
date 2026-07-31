from setuptools import find_packages, setup

package_name = "rover_slam_selector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/slam_selector.launch.py"]),
        ("share/" + package_name + "/config", ["config/slam_selector.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="samuel@example.com",
    description="Select indoor or exterior SLAM and publish one map->odom TF.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "slam_selector = rover_slam_selector.slam_selector_node:main",
        ],
    },
)
