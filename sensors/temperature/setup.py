import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'temperature'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vortex',
    maintainer_email='karate.martynas1@gmail.com',
    description='Temperature sensor data gartehring from temperature sensor system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "temperature_publisher_node = temperature.temperature_publisher_node:main"
        ],
    },
)
