import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vela_data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sessions'), glob('sessions/*.toml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filippo',
    maintainer_email='filippogorini7@gmail.com',
    description='Recording pipeline for Kinova Gen3 + Robotiq 2F-85 LeRobot datasets via lerobot_ros.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'record_session = vela_data_collection.record_session:main',
        ],
    },
)
