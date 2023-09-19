import os
from glob import glob
from setuptools import setup

package_name = 'opencv_tutorials'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='opencv_tutorials',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processing = opencv_tutorials.processing:main',
            'image_pub = opencv_tutorials.image_pub:main',
            'detect_pump = opencv_tutorials.detect_pump:main',
            'pump = opencv_tutorials.pump:main',
        ],
    },
)
