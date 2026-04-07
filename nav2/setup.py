from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # FIXED: correct path to launch files
        (os.path.join('share', package_name, 'launch'),
            glob('nav2/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='anirudh110106@gmail.com',
    description='Robot system with odometry and SLAM',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom = nav2.odom:main',
            'laser_node = nav2.laser:main',
        ],
    },
)