import glob

from setuptools import find_packages, setup

package_name = "mrobosub_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michigan Robotic Submarine",
    maintainer_email="michiganroboticsubmarine@gmail.com",
    description="TODO: Package description",
    license="BSD-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hal = mrobosub_sim.hal:main",
        ],
    },
)
