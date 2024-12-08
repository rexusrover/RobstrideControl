import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class VelocityControl(Node):
    def __init__(self):
        super().__init__('velocity_control')
        self.publisher = self.create_publisher(Float32, '/motor_velocity', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )
        self.get_logger().info('Velocity Control Node has started.')

    def joystick_callback(self, msg):
        # Right joystick vertical axis is usually index 4
        right_joystick_vertical = msg.axes[4]

        # Scale joystick input (-1 to 1) to velocity (e.g., -100 to 100)
        max_velocity = 20  # Adjust this value based on your motor's limits
        velocity = right_joystick_vertical * max_velocity

        # Publish velocity command
        velocity_msg = Float32()
        velocity_msg.data = velocity
        self.publisher.publish(velocity_msg)

        # Log the velocity
        self.get_logger().info(f"Velocity Command: {velocity:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
