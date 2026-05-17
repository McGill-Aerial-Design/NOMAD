from setuptools import setup
import os
from glob import glob

package_name = 'target_localizer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MAD Avionics',
    maintainer_email='avionics@mcgilldroneclub.com',
    description='AEAC 2026 Task 1 target localizer (HSV detection + GPS positioning)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'target_localizer_node = target_localizer.target_localizer_node:main',
        ],
    },
)
