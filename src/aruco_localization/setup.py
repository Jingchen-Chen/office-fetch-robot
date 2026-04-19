import os
from glob import glob
from setuptools import setup

package_name = 'aruco_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team P14',
    maintainer_email='p14@example.com',
    description='ARUCO marker-based localization for the Office Fetch Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_localization.aruco_detector:main',
            'pose_estimator = aruco_localization.pose_estimator:main',
        ],
    },
)
