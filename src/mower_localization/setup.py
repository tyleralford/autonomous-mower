from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mower_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tyler',
    maintainer_email='alford.tyler94@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_heading_node = mower_localization.ground_truth_heading_node:main',
            'recorder_node = mower_localization.recorder_node:main',
            'map_to_utm_broadcaster = mower_localization.map_to_utm_broadcaster:main'
        ],
    },
)
