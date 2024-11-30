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
    binary = f'{comm_type:08b}{0:08b}{host_can_id:08b}{motor_can_id:08b}'
    binary = binary + '100'  # Add '100' to the right
    binary = binary[3:]  # Drop the first 3 bits
    hex_value = f'{int(binary, 2):08x}'  # Convert back to hexadecimal
    return [int(hex_value[i:i+2], 16) for i in range(0, len(hex_value), 2)]


def build_command(comm_type, param_index, value=None, motor_can_id=127):
    """
    Build a full command based on the communication type, parameter index, motor CAN ID, and optional value.
    """
    header = [0x41, 0x54]
    tail = [0x0D, 0x0A]
    extended_header = build_extended_header(comm_type, 253, motor_can_id)
    data_length = [0x08]
    if comm_type == 18:  # Write a single parameter
        param_index_bytes = [int(param_index[:2], 16), int(param_index[2:], 16)]
        if value is not None:
            ieee_value = float_to_ieee754_hex(value)
            data_area = param_index_bytes + [0x00, 0x00] + [int(ieee_value[i:i+2], 16) for i in range(0, len(ieee_value), 2)][::-1]
        else:
            data_area = param_index_bytes + [0x00, 0x00, 0x01, 0x00, 0x00, 0x00]
    elif comm_type == 6:  # Reset position
        param_index_bytes = [0x01, 0x00]
        data_area = param_index_bytes + [0x00] * 6
    else:  # Communication types 3 and 4
        data_area = [0x00] * 8
    command = header + extended_header + data_length + data_area + tail
    return command


def send_command(ser, command):
    """
    Send a command via serial and display it in formatted hex.
    """
    formatted_command = ' '.join(f'{byte:02x}' for byte in command)
    print(f"Sent: {formatted_command}")
    ser.write(bytes(command))


def reset_position(ser, motor_can_id):
    """
    Reset the current position of a motor to zero.
    """
    speed_command = build_command(18, '1770', value=0.0, motor_can_id=motor_can_id)
    send_command(ser, speed_command)
    time.sleep(0.1)
    angle_command = build_command(18, '1670', value=0.0, motor_can_id=motor_can_id)
    send_command(ser, angle_command)
    time.sleep(0.1)
    reset_position_command = build_command(6, None, motor_can_id=motor_can_id)
    send_command(ser, reset_position_command)
    time.sleep(0.1)


def initialize_motor(ser, motor_can_id):
    """
    Initialize a motor for position mode.
    """
    position_mode_command = build_command(18, '0570', value=None, motor_can_id=motor_can_id)
    send_command(ser, position_mode_command)
    time.sleep(0.1)
    enable_motor_command = build_command(3, None, motor_can_id=motor_can_id)
    send_command(ser, enable_motor_command)
    time.sleep(0.1)


def main():
    # Configuration
    port = "COM7"
    baud_rate = 921600
    motor_ids = [1, 127]

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Opened {port} at {baud_rate} baud rate.")

            # Initialize both motors
            for motor_id in motor_ids:
                print(f"Initializing motor with CAN ID {motor_id}...")
                initialize_motor(ser, motor_id)

            # Main loop
            while True:
                try:
                    user_input = input(
                        "Enter CAN ID, target position, and speed (e.g., 127,10,5) or CAN ID,r to reset: ").strip()
                    if ',' not in user_input:
                        print("Invalid input format. Use CAN ID,Position,Speed or CAN ID,r.")
                        continue

                    parts = user_input.split(',')
                    if len(parts) == 2 and parts[1].lower() == 'r':
                        # Reset command
                        motor_can_id = int(parts[0])
                        if motor_can_id not in motor_ids:
                            print(f"Invalid CAN ID. Choose from {motor_ids}.")
                            continue
                        reset_position(ser, motor_can_id)
                    elif len(parts) == 3:
                        # Position and speed command
                        motor_can_id = int(parts[0])
                        if motor_can_id not in motor_ids:
                            print(f"Invalid CAN ID. Choose from {motor_ids}.")
                            continue
                        position = float(parts[1])
                        speed = float(parts[2])
                        if not (-44.0 <= speed <= 44.0):
                            print("Speed out of range. Please enter a value between -44 and 44 rad/s.")
                            continue
                        # Set target position and speed
                        speed_command = build_command(18, '1770', value=speed, motor_can_id=motor_can_id)
                        send_command(ser, speed_command)
                        time.sleep(0.1)
                        position_command = build_command(18, '1670', value=position, motor_can_id=motor_can_id)
                        send_command(ser, position_command)
                    else:
                        print("Invalid input format. Use CAN ID,Position,Speed or CAN ID,r to reset.")
                except ValueError:
                    print("Invalid input. Ensure values are correct and use the correct format.")
                except Exception as e:
                    print(f"Error: {e}")

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
