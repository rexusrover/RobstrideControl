import serial
import struct
import time


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


def build_command(comm_type, param_index, motor_can_id=127):
    """
    Build a read command based on the communication type, parameter index, and motor CAN ID.
    """
    header = [0x41, 0x54]
    tail = [0x0D, 0x0A]
    extended_header = build_extended_header(comm_type, 253, motor_can_id)
    data_length = [0x08]
    param_index_bytes = [int(param_index[:2], 16), int(param_index[2:], 16)]
    data_area = param_index_bytes + [0x00] * 6  # Remaining area is all zeros
    return header + extended_header + data_length + data_area + tail


def send_command(ser, command):
    """
    Send a command via serial.
    """
    ser.write(bytes(command))


def main():
    # Configuration
    port = "COM7"
    baud_rate = 921600
    motor_can_id = 127

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Opened {port} at {baud_rate} baud rate.")
            print("Reading encoder data... Press Ctrl+C to stop.")

            # Loop to continuously read encoder data
            while True:
                # Build and send read command
                read_command = build_command(17, '1970', motor_can_id=motor_can_id)
                send_command(ser, read_command)

                # Wait for a response
                time.sleep(0.1)  # Ensure proper response timing
                if ser.in_waiting > 0:
                    received_data = ser.read(ser.in_waiting)
                    if len(received_data) >= 14:  # Ensure response length is correct
                        encoder_data_hex = ''.join(f'{byte:02x}' for byte in received_data[-6:-2])  # Extract last 4 bytes
                        encoder_value = float_from_ieee754_hex(encoder_data_hex)
                        print(f"Encoder Position: {encoder_value:.4f}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
