import os
from glob import glob
from setuptools import setup

package_name = 'merge_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdulkadrtr',
    maintainer_email='abdulkadirthe@gmail.com',
    description='ROS2 Merge_Map package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'merge_map = merge_map.merge_map:main',
            'occupancy_viewer = merge_map.occupancy_viewer:main'
        ],
    },
)
