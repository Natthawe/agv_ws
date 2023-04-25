import os
from glob import glob
from setuptools import setup

package_name = 'moteus_vel'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'moteus'],
    zip_safe=True,
    maintainer='moteus',
    maintainer_email='chaiwit.p@groupmaker.co.th',
    description='moteus_vel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moteus_control_node = moteus_vel.moteus_control_node:main",
            "moteus_control_old = moteus_vel.moteus_control_node_old2:main",
            "test_odom = moteus_vel.test_odom:main"

        ],
    },
)