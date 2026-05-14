from setuptools import setup
import os
from glob import glob

package_name = 'weed_robot_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Zioni',
    maintainer_email='Daniel.zioni8@gmail.com',
    description='Hand-eye calibration pipeline for UR5 + RealSense D435i',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
