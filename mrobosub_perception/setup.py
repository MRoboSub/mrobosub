from setuptools import find_packages, setup
import glob

package_name = 'mrobosub_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f"share/{package_name}/launch", glob.glob("launch/*")),
        (f"share/{package_name}/params", glob.glob("params/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='muskaan@umich.edu',
    description='This is our fire code. It uses HSV and a YOLOV model. #mrobosub_perception',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_botcam_publisher = mrobosub_perception.dummy_botcam_publisher:main',
            'bin_hsv = mrobosub_perception.bin_hsv:main',
            'pathmarker_hsv= mrobosub_perception.pathmarker_hsv:main',
            'ml_executor = mrobosub_perception.ml_executor:main',
            'ml_srv = mrobosub_perception.ml_srv:main',
        ],
    },
)
