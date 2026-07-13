from setuptools import find_packages, setup
import glob


package_name = "mrobosub_hal"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*")),
        (f"share/{package_name}/params", glob.glob("params/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="Michigan Robotic Submarine",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thruster_controller = mrobosub_hal.thruster_controller:main",
                            "imu = mrobosub_hal.imu:main",
                  "dvl_publisher = mrobosub_hal.dvl_publisher:main",
                         "botcam = mrobosub_hal.botcam:main",
                            "zed = mrobosub_hal.zed:main",
                 "esp32_thruster = mrobosub_hal.esp32_thruster:main",
                        "arduino = mrobosub_hal.arduino:main"
        ],
    },
)
