import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'physical_ai_amr'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models', 'physical_ai_amr'),
            glob('models/physical_ai_amr/*')),
        (os.path.join('share', package_name, 'maps'),
            ['maps/patrol_world.pgm', 'maps/patrol_world.yaml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description=(
        'Physical AI AMR package with a Gazebo Harmonic differential-drive robot: '
        'reactive patrol/obstacle avoidance with Nav2 goal handoff.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_command_node = physical_ai_amr.robot_command_node:main',
        ],
    },
)
