from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'web'),
            glob(os.path.join(package_name, 'web', '*.*'))),
    ],
    include_package_data=True,
    package_data={
        package_name: ['web/*.*'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='Web dashboard for YahboomcarX3: device status + kinematics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = yahboomcar_dashboard.dashboard_node:main',
        ],
    },
)
