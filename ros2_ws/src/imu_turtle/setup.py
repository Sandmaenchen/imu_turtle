import os
from glob import glob
from setuptools import setup

package_name = 'imu_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandmann',
    maintainer_email='matti.kortelainen@uef.fi',
    description='Controlling turtlesim with IMU',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node1 = imu_turtle.node1:main',
            'node2 = imu_turtle.node2:main',
            'node3 = imu_turtle.node3:main',
        ],
    },
)
