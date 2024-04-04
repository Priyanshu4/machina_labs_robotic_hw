from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotic_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='priyanshu',
    maintainer_email='priyanshu.agrawal@uconn.edu',
    description='My solution to the Machina Labs robotic homework',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_cell_data_server = robotic_hw.load_cell_data_server:main',
            'load_cell_data_publisher = robotic_hw.load_cell_data_publisher:main',
        ],
    },
)
