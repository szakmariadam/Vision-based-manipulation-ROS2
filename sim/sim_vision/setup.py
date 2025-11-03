from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        #Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),

        #Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.npz')),

        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='Szakmari.adam@gmail.com',
    description='Simulation for computer vision',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_saver = sim_vision.image_saver:main',
            'extrinsic_calib = sim_vision.extrinsic_calib:main'
        ],
    },
)
