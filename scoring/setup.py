from setuptools import setup

package_name = 'scoring'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daksh Bhaskar',
    maintainer_email='dakshb2@illinois.edu',
    description='GT tap + trajectory-error scoring for the benchmark.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'pose_to_tum = scoring.pose_to_tum:main',
    ]},
)