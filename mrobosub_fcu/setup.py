from setuptools import find_packages, setup
import glob

package_name = "mrobosub_fcu"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="flight controller unit, offerring direct control over motors and sensors",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["thruster_mixing = mrobosub_fcu.thruster_mixing:main"],
    },
)
