import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'odom_wheel'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='Odometry_wheel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_wheel = odom_wheel.odom_wheel_node:main'
        ],
    },
)
