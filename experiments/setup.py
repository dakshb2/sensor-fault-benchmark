import os
from glob import glob
from setuptools import setup

package_name = 'experiments'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'trajectories'), glob('trajectories/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daksh Bhaskar',
    maintainer_email='dakshb2@illinois.edu',
    description='Trajectory driver and run scripts.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'drive_trajectory = experiments.drive_trajectory:main',
    ]},
)