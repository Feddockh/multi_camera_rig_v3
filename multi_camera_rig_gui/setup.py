from setuptools import setup
import os
from glob import glob

package_name = 'multi_camera_rig_gui'

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
    maintainer='Hayden',
    maintainer_email='hayden@todo.todo',
    description='GUI for multi-camera rig control and monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = multi_camera_rig_gui.gui_node:main',
        ],
    },
)
