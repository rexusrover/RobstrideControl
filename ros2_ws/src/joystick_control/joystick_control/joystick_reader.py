import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickReader(Node):
    def __init__(self):
        super().__init__('joystick_reader')
        self.subscription = self.create_subscription(
            Joy,  # Message type
            '/joy',  # Topic name
            self.joystick_callback,  # Callback function
            10  # QoS (queue size)
        )
        self.get_logger().info('Joystick Reader Node has started.')

    def joystick_callback(self, msg):
        # Right joystick axes: usually index 3 (horizontal) and 4 (vertical)
        right_joystick_horizontal = msg.axes[3]
        right_joystick_vertical = msg.axes[4]

        # Log joystick values
        self.get_logger().info(
            f"Right Joystick - Horizontal: {right_joystick_horizontal}, Vertical: {right_joystick_vertical}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoystickReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
