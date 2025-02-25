import serial
import time

# Define COM port and baud rate
COM_PORT = "COM17"  # Adjust based on your system
BAUD_RATE = 250000  # Default CH340 baud rate for CAN module

# Command to read the angle register (example: 0x11)
READ_ANGLE_CMD = bytes([0xFF, 0xAA, 0x27, 0x11, 0x00])

def main():
    try:
        # Open serial port
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Opened {COM_PORT} at {BAUD_RATE} baud")

            # Send command to request angle data
            ser.write(READ_ANGLE_CMD)
            print(f"Sent: {READ_ANGLE_CMD.hex().upper()}")

            # Listen for a response
            time.sleep(0.1)  # Small delay to allow response

            while ser.in_waiting:
                response = ser.read(8)  # Read up to 8 bytes (adjust based on expected reply size)
                print(f"Received: {response.hex().upper()}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    main()
