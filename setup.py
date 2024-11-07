import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_psutil'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cogni',
    maintainer_email='bperseghetti@rudislabs.com',
    description='psutil over ROS2 transport',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'psutil_node = ros2_psutil.psutil_node:main'
        ],
    },
)
