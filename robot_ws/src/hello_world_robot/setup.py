from setuptools import setup, find_packages

package_name = 'hello_world_robot'

setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/rotate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/deploy_rotate.launch.py']),
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

