import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import math
import serial
import threading


class JoystickPositionControl(Node):
    def __init__(self):
        super().__init__('joystick_position_control')

        # Subscribe to the joystick input topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )

        # Publish the computed motor position
        self.publisher = self.create_publisher(Float32, '/motor_position', 10)

        # Setup serial communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Start a separate thread to listen for Arduino responses
        self.serial_thread = threading.Thread(target=self.read_arduino_echo, daemon=True)
        self.serial_thread.start()

        self.dead_zone = 0.1  # Dead zone to filter noise
        self.get_logger().info("Joystick Position Control Node Started")

    def joystick_callback(self, msg):
        # Left joystick axes
        x = msg.axes[0]  # Horizontal axis
        y = msg.axes[1]  # Vertical axis

        # Apply dead zone to avoid noise
        if abs(x) < self.dead_zone and abs(y) < self.dead_zone:
            x = 0.0
            y = 0.0

        # Calculate target angle in radians using atan2 and normalize to [0, 2π]
        target_angle = math.atan2(y, x)  # Range: [-pi, pi]
        if target_angle < 0:
            target_angle += 2 * math.pi  # Normalize to [0, 2π]

        # Current angle normalization (replace with actual motor feedback if available)
        current_angle = 0.0  # Placeholder: replace with feedback from the motor
        current_angle = current_angle % (2 * math.pi)  # Normalize to [0, 2π]

        # Compute the shortest path to the target angle
        delta_angle = target_angle - current_angle

        # Normalize delta_angle to [-pi, pi] to ensure shortest path
        if delta_angle > math.pi:
            delta_angle -= 2 * math.pi
        elif delta_angle < -math.pi:
            delta_angle += 2 * math.pi

        # Compute the final target angle within [0, 2π]
        final_target_angle = (current_angle + delta_angle) % (2 * math.pi)

        # Convert angles to degrees for logging
        target_angle_degrees = math.degrees(target_angle)
        current_angle_degrees = math.degrees(current_angle)
        final_target_angle_degrees = math.degrees(final_target_angle)

        # Log the angles in both radians and degrees
        self.get_logger().info(
            f"Joystick position: x={x:.2f}, y={y:.2f}, "
            f"target_angle={target_angle:.2f} radians ({target_angle_degrees:.2f} degrees), "
            f"current_angle={current_angle:.2f} radians ({current_angle_degrees:.2f} degrees), "
            f"final_target_angle={final_target_angle:.2f} radians ({final_target_angle_degrees:.2f} degrees)"
        )

        # Publish the final target angle
        angle_msg = Float32()
        angle_msg.data = final_target_angle
        self.publisher.publish(angle_msg)

        # Send the final target angle to the Arduino
        self.send_to_arduino(final_target_angle)


    def send_to_arduino(self, angle):
        # Format the angle as a string and send it over serial
        command = f"{angle:.4f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

    def read_arduino_echo(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                # Read the echo from Arduino
                response = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Echo from Arduino: {response}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        node.serial_port.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
