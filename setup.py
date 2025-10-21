import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rovesugv_navsim'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'images'), glob('images/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/frami'), glob('models/frami/*')),
        (os.path.join('share', package_name, 'models/rover'), glob('models/rover/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name), glob('README.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannu Hakalahti',
    maintainer_email='hannu.hakalahti@seamk.fi',
    description='Simulate the GPS navigation of the four-wheeled rover',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_waypoint_commander = rovesugv_navsim.gps_waypoint_commander:main',
            'gui_waypoint_commander = rovesugv_navsim.gui_waypoint_commander:main',
        ],
    },
)
