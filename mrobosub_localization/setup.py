from setuptools import find_packages, setup
import glob

package_name = "mrobosub_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="synthesis of sensor information to determine robot state",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["localization = mrobosub_localization.localization:main"],
    },
)
