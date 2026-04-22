from setuptools import find_packages, setup

package_name = 'reachy_BoMI_ROS2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elisa Martinenghi',
    maintainer_email='s6504193@studenti.unige.it',
    description='ROS 2 package for BoMI finger input to Reachy velocity commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_socket = reachy_BoMI_ROS2.server_socket:main',
            'cmd_vel_publisher = reachy_BoMI_ROS2.cmd_vel_publisher:main',
        ],
    },
)
