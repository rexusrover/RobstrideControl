import serial
import time
import threading


def send_command(ser, command):
    """Send a command to the motor and log it."""
    ser.write(command)
    print(f"Sent: {command.hex()}")


def parse_command(input_command):
    """
    Parse a hex string command into a bytes object.
    - input_command: String of hexadecimal values separated by spaces.
    """
    try:
        return bytes.fromhex(input_command.strip())
    except ValueError:
        print("Invalid command format. Make sure it's a valid hex string.")
        return None


def listen_for_data(ser):
    """
    Continuously listen for incoming data on the serial port.
    """
    while True:
        if ser.in_waiting > 0:
            received_data = ser.read(ser.in_waiting)
            print(f"Received: {received_data.hex()}")
        time.sleep(0.1)  # Small delay to avoid excessive CPU usage


def main():
    # Configuration
    port = "COM3"  # Adjust this based on your setup
    baud_rate = 921600

    try:
        # Open the serial port
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Opened {port} at {baud_rate} baud rate.")

            # Start the listener thread
            listener_thread = threading.Thread(target=listen_for_data, args=(ser,), daemon=True)
            listener_thread.start()

            while True:
                # Get commands from the user
                command_input = input("Enter commands (comma-separated hex strings): ").strip()
                commands = command_input.split(",")

                for command_str in commands:
                    command = parse_command(command_str)
                    if command:
                        send_command(ser, command)
                        time.sleep(0.1)  # Small delay between commands

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting program...")


if __name__ == "__main__":
    main()
