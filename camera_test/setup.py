from glob import glob
import os
from setuptools import setup

package_name = 'camera_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='Test package to view camera feed',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    entry_points={
        'console_scripts': [
            'show_camera = camera_test.show_camera:main',
            'capture_dataset = camera_test.capture_dataset:main'
        ],
    },
)
