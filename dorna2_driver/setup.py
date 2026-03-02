import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dorna2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'dorna2'],
    zip_safe=True,
    maintainer='Hannah Wimpy',
    maintainer_email='hwimpy@lila.ai',
    description='ROS 2 driver node for Dorna 2 series robot arms',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dorna2_node = dorna2_driver.dorna2_node:main',
        ],
    },
)
