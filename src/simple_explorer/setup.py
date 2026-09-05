from setuptools import find_packages, setup

import os
from glob import glob

package_name = "simple_explorer"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"],
    ),
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
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="jleib",
    description="Simple frontier exploration demonstration",
    license="TODO",
    entry_points={
        "console_scripts": [
            "simple_frontier_explorer = "
            "simple_explorer.simple_frontier_explorer:main",
        ],
    },
)
