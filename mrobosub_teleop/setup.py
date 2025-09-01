import glob
from setuptools import find_packages, setup

package_name = "mrobosub_teleop"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*")),
        ("share/" + package_name + "/params", glob.glob("params/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="manual sub control",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["all_dof_teleop = mrobosub_teleop.all_dof_teleop:run"],
    },
)
