from setuptools import setup
import os
from glob import glob

package_name = 'gate_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@ucsd.edu',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_detector_node = gate_detection.gate_detector_node:main',
        ],
    },
)
