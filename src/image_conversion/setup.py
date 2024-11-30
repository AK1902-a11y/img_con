from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_conversion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files in the installation
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshat1902',
    maintainer_email='akshat1902@todo.todo',
    description='A ROS2 package for image conversion with dynamic mode switching.',
    license='Apache License 2.0',  # Replaced TODO with a valid license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Entry point for your image_conversion node
            'image_conversion = image_conversion.image_conversion:main',
        ],
    },
)

