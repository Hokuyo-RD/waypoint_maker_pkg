from setuptools import setup

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # パッケージ名を追加
    py_modules=['src.waypoint_manager'],
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
        ('share/' + package_name, ['package.xml']), # package.xml を追加
    ],
    entry_points={
        'console_scripts': [
            'waypoint_manager = src.waypoint_manager:main',
        ],
    },
)
