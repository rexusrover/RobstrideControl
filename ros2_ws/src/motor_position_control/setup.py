from setuptools import setup

package_name = 'motor_position_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Joystick-based motor position control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_position_control = motor_position_control.joystick_position_control:main',
        ],
    },
)
