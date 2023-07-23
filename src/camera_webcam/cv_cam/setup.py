from setuptools import setup
from glob import glob
import os

package_name = 'cv_cam'

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
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='CV CAMERA',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_cam = cv_cam.cv_cam_node:main',
            'cv2_cam = cv_cam.cv2_cam_node:main',
            'follower = cv_cam.follower_node:main',
            'test_topic = cv_cam.test_topic:main',
            'follower_pid = cv_cam.follower_pid:main',
            'linelaser = cv_cam.linelaser:main',
            'red_line = cv_cam.red_line_node:main',
        ],
    },
)
