import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading


class SerialInterface(Node):
    def __init__(self):
        super().__init__('serial_interface')
        self.subscription = self.create_subscription(
            Float32,
            '/motor_velocity',
            self.velocity_callback,
            10
        )
        # Configure serial connection (adjust port and baud rate as needed)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info('Serial Interface Node has started.')

        # Start a separate thread to listen for Arduino responses
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()

    def velocity_callback(self, msg):
        velocity = msg.data
        # Send velocity as a string to the Arduino
        command = f"VEL:{velocity:.2f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

    def read_serial_data(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Response from Arduino: {response}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.serial_port.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
