from glob import glob
import os
from setuptools import setup

package_name = 'gate_navigation'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrewyin',
    maintainer_email='andrewyingo@gmail.com',
    description='Gate navigation package using YOLO detection and pure pursuit control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_navigator = gate_navigation.gate_navigator:main',
        ],
    },
)
