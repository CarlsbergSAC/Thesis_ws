import os
from glob import glob
from setuptools import setup

package_name = 'swarm_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'config'), glob('config/*'))
    ],
    install_requires=['setuptools', 'cartographer_ros'],
    zip_safe=True,
    author='Luke Watson',
    author_email='lwat6554@uni.sydney.edu.au',
    maintainer='Luke Watson',
    maintainer_email='lwat6554@uni.sydney.edu.au',
    license='apache-2.0',
    description='Script launching SLAM under different namespaces for ros2swarm',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_robot_slam = multi_robot_slam.multi_robot_slam_node:main',
            'explore = swarm_slam.wavefront_frontier:main'
        ],
    },
)
