from setuptools import setup
import os
from glob import glob

package_name = 'robotcan_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hassan',
    maintainer_email='202350496@jbnu.ac.kr',
    description='PyQt GUI for robot CAN control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'can_gui = robotcan_gui.can_gui:main',
        ],
    },
)