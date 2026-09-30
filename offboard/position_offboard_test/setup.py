from glob import glob

from setuptools import setup

package_name = 'position_offboard_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PX4 Offboard Position Control Example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = position_offboard_test.offboard_control:main',
            'pp_waypoint_offboard = position_offboard_test.pp_waypoint_offboard:main',
        ],
    },
)
