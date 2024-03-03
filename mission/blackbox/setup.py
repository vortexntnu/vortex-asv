import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'blackbox'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vortex',
    maintainer_email='rose.j.kapps@gmail.com',
    description='Logs all ROS2 data and other internal statuses to a data file for use in analizing data later',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blackbox_node = blackbox.blackbox_node:main',
        ],
    },
)
