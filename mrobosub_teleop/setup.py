from setuptools import find_packages, setup

package_name = "mrobosub_teleop"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="manual sub control",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
