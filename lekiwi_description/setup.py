from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lekiwi_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 🔥 Install URDF files
        (os.path.join('share', package_name, 'URDF'),
            glob('URDF/*.urdf')),

        # 🔥 Install Xacro files
        (os.path.join('share', package_name, 'URDF'),
            glob('URDF/*.xacro')),

        # 🔥 Install meshes
        (os.path.join('share', package_name, 'URDF', 'meshes'),
            glob('URDF/meshes/*')),

        # 🔥 Install ros2_control files
        (os.path.join('share', package_name, 'ros2_control'),
            glob('ros2_control/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='anirudh110106@gmail.com',
    description='LeKiwi robot description package',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
