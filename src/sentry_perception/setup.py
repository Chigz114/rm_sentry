from setuptools import setup
from glob import glob
import os

package_name = 'sentry_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arch',
    maintainer_email='arch@sentry.local',
    description='RoboMaster Sentry perception nodes (obstacle detection on 3D lidar).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = sentry_perception.obstacle_detector_node:main',
            'cluster_detector  = sentry_perception.cluster_detector_node:main',
            'esdf2d            = sentry_perception.esdf2d_node:main',
            'relocalization    = sentry_perception.relocalization_node:main',
        ],
    },
)
