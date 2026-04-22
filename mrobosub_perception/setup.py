from setuptools import find_packages, setup
import glob
import os

package_name = 'mrobosub_perception'

def get_data_files(src_dir, dest_prefix):
    result = []
    for dirpath, dirnames, filenames in os.walk(src_dir):
        dirnames[:] = [d for d in dirnames if d != '__pycache__']
        if not filenames:
            continue
        files = [os.path.join(dirpath, f) for f in filenames if not f.endswith('.pyc')]
        dest = os.path.join(dest_prefix, dirpath)
        result.append((dest, files))
    return result

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
        *get_data_files('yolov5', 'share/' + package_name),
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
            'rectify_image = mrobosub_perception.rectify_image:main',
            'dummy_ml_srv_node = mrobosub_perception.dummy_ml_srv_node:main',
            'png_pub = mrobosub_perception.png_pub:main',
        ],
    },
)
