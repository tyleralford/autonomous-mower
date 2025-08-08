from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mower_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tyler',
    maintainer_email='alford.tyler94@gmail.com',
    description='Top-level launch files for the autonomous mower system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_teleop = mower_bringup.simple_teleop:main',
        ],
    },
)
