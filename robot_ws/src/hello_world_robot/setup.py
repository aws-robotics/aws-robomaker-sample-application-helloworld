from setuptools import find_packages, setup

package_name = 'hello_world_robot'

from ament_index_python.packages import get_package_share_directory
from setuptools.command.install import install
from shutil import copyfile
import os

package_name = 'hello_world_robot'

class CopyRvizModel(install):
    def run(self):
        src = get_package_share_directory('turtlebot3_description')+'/rviz/model.rviz'
        dest_dir = '../../install/hello_world_robot/rviz'
        os.mkdir(dest_dir)
        copyfile(src, dest_dir+'/turtlebot3_description.rviz')
        install.run(self)

setup(
    name=package_name,
    version='2.2.1',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    cmdclass={
        'install': CopyRvizModel,
    },
    data_files=[
        ('share/' + package_name + '/launch', ['launch/rotate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/deploy_rotate.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'AWS RoboMaker robot package with a rotating Turtlebot3'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate = hello_world_robot.rotate:main',
        ],
    },
)
