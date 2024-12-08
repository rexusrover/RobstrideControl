from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the ROS 2 joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # Start the joystick reader node
        Node(
            package='joystick_control',
            executable='joystick_reader',
            name='joystick_reader_node',
            output='screen'
        ),
        # Start the velocity control node
        Node(
            package='joystick_control',
            executable='velocity_control',
            name='velocity_control_node',
            output='screen'
        ),
        # Start the serial interface node
        Node(
            package='joystick_control',
            executable='serial_interface',
            name='serial_interface_node',
            output='screen'
        ),
    ])
