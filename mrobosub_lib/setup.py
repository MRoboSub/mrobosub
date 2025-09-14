from setuptools import find_packages, setup

from mrobosub_teleop.mrobosub_teleop import joystick_teleop

package_name = "mrobosub_lib"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="The mrobosub_lib package",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
