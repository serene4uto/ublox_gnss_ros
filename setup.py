from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ublox_gnss_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch dir
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config dir
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ublox_gnss_node = src.ublox_gnss_node:main",
        ],
    },
)
