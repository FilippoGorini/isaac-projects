from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vla_policy_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filippo',
    maintainer_email='filippogorini7@gmail.com',
    description='ROS 2 policy client for pi0 VLA on UR5e',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'policy_client = vla_policy_client.policy_client_node:main',
        ],
    },
)
