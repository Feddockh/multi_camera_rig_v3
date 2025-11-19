from setuptools import setup
import os
from glob import glob

package_name = 'multi_camera_rig_trigger'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Hardware trigger controller for multi-camera rig system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_node = multi_camera_rig_trigger.trigger_node:main',
        ],
    },
)
