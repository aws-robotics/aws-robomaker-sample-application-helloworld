import os
from shutil import copyfile

from ament_index_python.packages import get_package_share_directory
from setuptools import find_packages, setup
from setuptools.command.install import install


package_name = 'hello_world_robot'


class CopyRvizModelToPackageDir(install):

    def run(self):
        rviz_file = get_package_share_directory('turtlebot3_description') \
            + '/rviz/model.rviz'
        dest_dir = 'rviz'  # this is relative to hello_world_robot package directory
        if not os.path.isdir(dest_dir):
            os.mkdir(dest_dir)
        copyfile(rviz_file, dest_dir+'/turtlebot3_model.rviz')
        install.run(self)


setup(
    name=package_name,
    version='2.2.1',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    cmdclass={
        'install': CopyRvizModelToPackageDir,
    },
    data_files=[
        ('share/' + package_name + '/launch', ['launch/rotate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/deploy_rotate.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('rviz/', ['rviz/turtlebot3_model.rviz'])
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
