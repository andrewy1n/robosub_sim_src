import glob
import os
from setuptools import find_packages, setup

package_name = 'robosub_pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.*')),
        (os.path.join('share', package_name, 'scripts'), glob.glob('scripts/*.*')),
    ],
    install_requires=['setuptools', 'dave_interfaces'],
    zip_safe=True,
    maintainer='andrewyin',
    maintainer_email='andrewyingo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
            entry_points={
            'console_scripts': [
                'pid_controller = robosub_pid_controller.pid_controller:main',
                'pid_tuner = robosub_pid_controller.pid_tuner:main',
                'floating_demo = robosub_pid_controller.floating_demo:main',
                'test_position_orientation_hold = robosub_pid_controller.test_position_orientation_hold:main',
            ],
        },
)
