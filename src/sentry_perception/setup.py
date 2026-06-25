from setuptools import setup

package_name = 'sentry_perception'

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
    maintainer='arch',
    maintainer_email='arch@sentry.local',
    description='RoboMaster Sentry relocalization and 2D ESDF nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esdf2d            = sentry_perception.esdf2d_node:main',
            'relocalization    = sentry_perception.relocalization_node:main',
        ],
    },
)
