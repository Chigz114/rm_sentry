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
    description='Sentry Phase-3 trajectory tracker and keyboard teleop tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = sentry_controller.keyboard_teleop:main',
            'traj_tracker    = sentry_controller.traj_tracker:main',
        ],
    },
)
