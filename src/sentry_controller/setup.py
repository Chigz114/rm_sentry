from setuptools import setup

package_name = 'sentry_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@robomaster.local',
    description='Sentry Phase-3 controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = sentry_controller.keyboard_teleop:main',
            'goal_controller = sentry_controller.goal_controller:main',
            'path_tracker    = sentry_controller.path_tracker:main',
            'traj_tracker    = sentry_controller.traj_tracker:main',
        ],
    },
)
