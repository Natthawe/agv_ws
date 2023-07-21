from setuptools import setup
from glob import glob
import os


package_name = 'detect_obstacles'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gmc_cg',
    maintainer_email='natthawejumjai@gmail.com',
    description='Detect Obstacles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_obstacles_node = detect_obstacles.detect_obstacles_node:main',
            'range_node = detect_obstacles.range_node:main',
        ],
    },
)
