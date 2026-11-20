from setuptools import setup

package_name = "smart_car_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartCar contributors",
    maintainer_email="smart-car@users.noreply.github.com",
    description="LaserScan-based obstacle avoidance for SmartCar.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_avoidance = smart_car_control.obstacle_avoidance:main",
        ],
    },
)
