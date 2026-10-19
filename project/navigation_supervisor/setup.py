from setuptools import find_packages, setup

package_name = "navigation_supervisor"

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
            "share/" + package_name + "/config",
            ["config/nav2_unknown_space.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jleib",
    maintainer_email="justinleibbrandt.github@gmail.com",
    description=("Navigation supervision, recovery, and failed-goal handling."),
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "navigation_supervisor = "
            "navigation_supervisor.navigation_supervisor:main",
        ],
    },
)
