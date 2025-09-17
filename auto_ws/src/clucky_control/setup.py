import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'clucky_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozy',
    maintainer_email='daeganbrown03@gmail.com',
    description='Bridge node between actions_cpp and nav2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_sender = clucky_control.goal_sender:main',
            'cmd_vel_bridge = clucky_control.cmd_vel_bridge:main',
        ],
    },
)
