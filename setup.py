from setuptools import setup

package_name = 'waypoint_maker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'src.waypoint_maker',  # ← 'src.' を追加
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 waypoint maker',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_maker = src.waypoint_maker:main',  # ← 'src.' を追加
        ],
    },
)