from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    py_modules=[
        # 既存のモジュールに加えて新しいモジュールを追加
        'src.waypoint_manager',
        'src.nav2_executer'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takahashi Shotaro',
    maintainer_email='s-takahashi@hokuyo-aut.co.jp',
    description='ROS 2 waypoint manager',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            # 既存のノードに加えて新しいノードを追加
            'waypoint_manager = src.waypoint_manager:main',
            'nav2_executer = src.nav2_executer:main'
        ],
    },
)