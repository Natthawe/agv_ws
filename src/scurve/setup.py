from setuptools import setup

package_name = 'scurve'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include our package.xml file
        # (os.path.join('share', package_name), ['package.xml']),
        # # Include all launch files.
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            's_curve = scurve.s_curve:main'
        ],
    },
)
