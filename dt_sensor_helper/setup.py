import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dt_sensor_helper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhkw0001',
    maintainer_email='henrik.lurz@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simplify_laser_scan_node = dt_sensor_helper.simplify_laser_scan:main",
            "simplify_laser_scan_showcase_node = dt_sensor_helper.simplify_laser_scan_showcase:main",
        ],
    },
)
