import os
from glob import glob
from setuptools import setup

package_name = 'door_tof_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
         # Include all config files.   
        (os.path.join('share', package_name), glob('config/*config.rviz')), 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srb',
    maintainer_email='sbaruco@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "machine_ControlNode = door_tof_sensor.machine_ControlNode:main",
            "door_dynamic_tf2_transform = door_tof_sensor.door_dynamic_tf2_transform:main", 
        ],
    },
)
