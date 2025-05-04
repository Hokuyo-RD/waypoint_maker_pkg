from setuptools import setup

package_name = 'waypoint_maker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # パッケージ名を追加
    py_modules=['src.waypoint_maker'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 waypoint maker',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), # package.xml を追加
    ],
    entry_points={
        'console_scripts': [
            'waypoint_maker = src.waypoint_maker:main',
        ],
    },
)
