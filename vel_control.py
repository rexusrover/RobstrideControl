import serial
import struct
import time


def float_to_ieee754_hex(value):
    """
    Convert a float to IEEE754 single-precision format and return as a hex string (big-endian).
    """
    ieee754 = struct.pack('<f', value)  # Convert to IEEE754 binary
    return ''.join(f'{byte:02x}' for byte in ieee754[::-1])  # Flip for big-endian


def build_extended_header(comm_type, host_can_id, motor_can_id):
    """
    Build the extended header from communication type, host CAN ID, and motor CAN ID.
    """
    # Combine into binary: comm_type (8 bits) + meaningless 0 (8 bits) + host_can_id (8 bits) + motor_can_id (8 bits)
    binary = f'{comm_type:08b}{0:08b}{host_can_id:08b}{motor_can_id:08b}'
    binary = binary + '100'  # Add '100' to the right
    binary = binary[3:]  # Drop the first 3 bits
    hex_value = f'{int(binary, 2):08x}'  # Convert back to hexadecimal
    return [int(hex_value[i:i+2], 16) for i in range(0, len(hex_value), 2)]  # Return as list of bytes


def build_command(comm_type, param_index, motor_can_id, value=None):
    """
    Build a full command based on the communication type, parameter index, motor CAN ID, and optional value.
    """
    # Header and Tail
    header = [0x41, 0x54]
    tail = [0x0D, 0x0A]

    # Extended Header
    extended_header = build_extended_header(comm_type, 253, motor_can_id)  # Host ID = 253

    # Data Area
    data_length = [0x08]
    if comm_type == 18:  # Write a single parameter
        param_index_bytes = [int(param_index[:2], 16), int(param_index[2:], 16)]
        if value is not None:  # Handle parameter value
            ieee_value = float_to_ieee754_hex(value)
            data_area = param_index_bytes + [0x00, 0x00] + [int(ieee_value[i:i+2], 16) for i in range(0, len(ieee_value), 2)][::-1]
        else:  # Velocity mode case
            data_area = param_index_bytes + [0x00, 0x00, 0x02, 0x00, 0x00, 0x00]
    else:  # Communication types 3 and 4
        data_area = [0x00] * 8

    # Combine to build the full command
    command = header + extended_header + data_length + data_area + tail
    return command


def send_command(ser, command):
    """
    Send a command via serial and display it in formatted hex.
    """
    formatted_command = ' '.join(f'{byte:02x}' for byte in command)
    print(f"Sent: {formatted_command}")
    ser.write(bytes(command))


def initialize_motor(ser, motor_can_id):
    """
    Initialize a motor: set to velocity mode, enable motor, and set max current.
    """
    # Step 1: Set motor to velocity mode
    velocity_mode_command = build_command(18, '0570', motor_can_id, value=None)
    send_command(ser, velocity_mode_command)
    time.sleep(0.1)

    # Step 2: Enable the motor
    enable_motor_command = build_command(3, None, motor_can_id)
    send_command(ser, enable_motor_command)
    time.sleep(0.1)

    # Step 3: Set max current
    max_current_command = build_command(18, '1870', motor_can_id, value=23.0)
    send_command(ser, max_current_command)
    time.sleep(0.1)


def main():
    # Configuration
    port = "COM7"
    baud_rate = 921600

    try:
        # Open serial connection
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Opened {port} at {baud_rate} baud rate.")

            # Initialize both motors
            print("Initializing motor with CAN ID 127...")
            initialize_motor(ser, 127)
            print("Initializing motor with CAN ID 1...")
            initialize_motor(ser, 1)

            # Main loop to set speed
            while True:
                try:
                    user_input = input("Enter CAN ID and target speed separated by a comma (e.g., 127,10): ").strip()
                    can_id_str, speed_str = user_input.split(',')
                    motor_can_id = int(can_id_str)
                    speed = float(speed_str)

                    if motor_can_id not in [1, 127]:
                        print("Invalid CAN ID. Please enter 1 or 127.")
                        continue

                    if not (-44.0 <= speed <= 44.0):
                        print("Speed out of range. Please enter a value between -44 and 44 rad/s.")
                        continue

                    # Build and send speed command
                    speed_command = build_command(18, '0a70', motor_can_id, value=speed)
                    send_command(ser, speed_command)

                except ValueError:
                    print("Invalid input. Please enter CAN ID and speed separated by a comma.")

                # Receive data from motor
                if ser.in_waiting > 0:
                    received_data = ser.read(ser.in_waiting)
                    formatted_received = ' '.join(f'{byte:02x}' for byte in received_data)
                    print(f"Received: {formatted_received}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
