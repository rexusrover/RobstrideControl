import serial
import struct
import time


def float_to_ieee754_hex(value):
    """
    Convert a float to IEEE754 single-precision format and return as a hex string (little-endian).
    """
    ieee754 = struct.pack('<f', value)  # Convert to IEEE754 binary (little-endian)
    return ''.join(f'{byte:02x}' for byte in ieee754)  # Convert to hex string


def float_from_ieee754_hex(hex_value):
    """
    Convert a hex string representing IEEE754 little-endian single-precision float to a float.
    """
    binary_value = bytes.fromhex(hex_value)  # Convert hex string to binary
    return struct.unpack('<f', binary_value)[0]  # Interpret as little-endian float


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
            ieee_value = float_to_ieee754_hex(value)  # IEEE754 in little-endian
            data_area = param_index_bytes + [0x00, 0x00] + [int(ieee_value[i:i+2], 16) for i in range(0, len(ieee_value), 2)]
        else:
            data_area = param_index_bytes + [0x00, 0x00, 0x02, 0x00, 0x00, 0x00]
    elif comm_type == 6:  # Reset position
        param_index_bytes = [0x01, 0x00]
        data_area = param_index_bytes + [0x00] * 6
    elif comm_type == 17:  # Read command
        param_index_bytes = [int(param_index[:2], 16), int(param_index[2:], 16)]
        data_area = param_index_bytes + [0x00] * 6  # Remaining area is all zeros
    else:
        data_area = [0x00] * 8

    return header + extended_header + data_length + data_area + tail


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
    Initialize a motor for speed mode.
    """
    speed_mode_command = build_command(18, '0570', value=None, motor_can_id=motor_can_id)
    send_command(ser, speed_mode_command)
    time.sleep(0.1)
    enable_motor_command = build_command(3, None, motor_can_id=motor_can_id)
    send_command(ser, enable_motor_command)
    time.sleep(0.1)


def main():
    # Configuration
    port = "COM7"
    baud_rate = 921600
    motor_can_id = 127

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Opened {port} at {baud_rate} baud rate.")

            # Step 1: Reset mech position to zero
            print("Resetting position to 0...")
            reset_position(ser, motor_can_id)

            # Step 2: Set speed mode and enable motor
            print("Initializing motor...")
            initialize_motor(ser, motor_can_id)

            # Step 3: Set max current
            max_current_command = build_command(18, '1870', value=23.0, motor_can_id=motor_can_id)
            send_command(ser, max_current_command)
            time.sleep(0.1)

            # Step 4: User sets speed
            while True:
                try:
                    speed = float(input("Enter target speed (-44 to 44 rad/s): "))
                    if not (-44.0 <= speed <= 44.0):
                        print("Speed out of range. Please enter a value between -44 and 44 rad/s.")
                        continue
                    speed_command = build_command(18, '0a70', value=speed, motor_can_id=motor_can_id)
                    send_command(ser, speed_command)
                    break
                except ValueError:
                    print("Invalid input. Please enter a valid number.")

            # Clear the input buffer to avoid reading old data
            ser.reset_input_buffer()

            # Step 5: Read encoder data indefinitely
            print("Reading encoder data...")
            while True:
                # Build and send read command
                read_command = build_command(17, '1970', motor_can_id=motor_can_id)
                send_command(ser, read_command)

                # Wait for a response
                time.sleep(0.1)  # Ensure proper response timing
                if ser.in_waiting > 0:
                    received_data = ser.read(ser.in_waiting)
                    formatted_received = ' '.join(f'{byte:02x}' for byte in received_data)
                    print(f"Received: {formatted_received}")

                    # Extract and decode encoder data (last 4 bytes before tail)
                    if len(received_data) >= 14:  # Ensure response length is correct
                        extracted_data = formatted_received.split()[-6:-2]  # Last 4 bytes before 0D 0A
                        print(f"Extracted Data: {extracted_data}")  # Debug extracted bytes
                        encoder_data_hex = ''.join(extracted_data)  # Keep as-is without reversal
                        print(f"Hex for IEEE754 Decoding: {encoder_data_hex}")  # Debug hex
                        encoder_value = float_from_ieee754_hex(encoder_data_hex)  # Decode directly
                        print(f"Encoder Position: {encoder_value:.4f}")
                else:
                    print("No data received. Retrying...")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
