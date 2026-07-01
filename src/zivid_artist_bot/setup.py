from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zivid_artist_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mark@todo.todo',
    description='Zivid Artist Bot using rembg and continuous raycasting',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_and_create_svg = zivid_artist_bot.svg_converter:main',
            'single_joint_test = zivid_artist_bot.single_joint_test:main',
            'cartesian_control_test = zivid_artist_bot.cartesian_control_test:main',

        ],
    },
)
