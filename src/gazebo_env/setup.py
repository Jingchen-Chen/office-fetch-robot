from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_env'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # Models need to be copied with their directory structure
        (os.path.join('share', package_name, 'models/target_object'), glob('models/target_object/*')),
        (os.path.join('share', package_name, 'models/aruco_marker'), glob('models/aruco_marker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gazebo simulation environment for Office Fetch Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
