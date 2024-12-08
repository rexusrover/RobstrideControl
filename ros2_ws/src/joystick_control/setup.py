from setuptools import setup

package_name = 'joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include the launch directory
        ('share/' + package_name + '/launch', ['launch/control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Joystick control node for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_reader = joystick_control.joystick_reader:main',
            'velocity_control = joystick_control.velocity_control:main',
            'serial_interface = joystick_control.serial_interface:main',
        ],
    },
)
