import serial
import pygame
import time

# Serial setup
SERIAL_PORT = "COM5"  # Change as needed
BAUD_RATE = 115200
ACK_TIMEOUT = 0.5  # Seconds to wait for acknowledgment
RETRY_LIMIT = 3  # Max retries before stopping

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Check if joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect one and restart.")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Define button mappings
J3_FWD_BUTTON = 3  # Triangle (Forward Joint 3)
J3_REV_BUTTON = 2  # Square (Reverse Joint 3)
J6_FWD_BUTTON = 6  # Triangle (Forward Joint 3)
J6_REV_BUTTON = 7  # Square (Reverse Joint 3)
J7_FWD_BUTTON = 0  # X
J7_REV_BUTTON = 1  # O
J1_FWD_AXIS = 5  # R2 (Forward Joint 1)
J1_REV_AXIS = 4  # L2 (Reverse Joint 1)
J2_FWD_HAT = "UP"  # D-pad UP (Forward Joint 2)
J2_REV_HAT = "DOWN"  # D-pad DOWN (Reverse Joint 2)
J4_FWD_HAT = "RIGHT"  # D-pad RIGHT (Forward Joint 4)
J4_REV_HAT = "LEFT"  # D-pad LEFT (Reverse Joint 4)
J5_FWD_BUTTON = 4 #L1
J5_REV_BUTTON = 5 #R1
DEADZONE = -0.8  # L2/R2 default is -1, ignore values between -1 and -0.8

# Track button states
button_states = {J3_FWD_BUTTON: False, J3_REV_BUTTON: False, J5_FWD_BUTTON: False, J5_REV_BUTTON: False, J7_FWD_BUTTON: False, J7_FWD_BUTTON: False, J6_FWD_BUTTON: False, J6_FWD_BUTTON: False}
axis_states = {J1_FWD_AXIS: False, J1_REV_AXIS: False}
hat_states = {J2_FWD_HAT: False, J2_REV_HAT: False, J4_FWD_HAT: False, J4_REV_HAT: False}

# Track last sent command per joint
last_sent_command = {"J3": None, "J1": None, "J2": None, "J4": None, "J5": None, "J6": None, "J7": None}

# Function to send command with acknowledgment tracking
def send_command_with_ack(command, joint):
    for attempt in range(RETRY_LIMIT):
        ser.write((command + "\n").encode())
        print(f"Sent: {command} (Attempt {attempt + 1})")

        start_time = time.time()
        while time.time() - start_time < ACK_TIMEOUT:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Received: {response}")
                
                if response == f"ACK_{command}":
                    return True  # Acknowledgment received

        print(f"Warning: No ACK received for {command}, retrying...")

    print(f"Error: Failed to receive ACK for {command} after {RETRY_LIMIT} retries.")
    return False  # Acknowledgment not received

try:
    print("PlayStation Controller Ready: Controlling Joint 1, Joint 2, Joint 3, and Joint 4")
    print("R2 (Axis 5) to move J1 forward, L2 (Axis 4) to move J1 backward.")
    print("D-pad UP to move J2 forward, D-pad DOWN to move J2 backward.")
    print("Triangle (3) to move J3 forward, Square (2) to move J3 backward.")
    print("D-pad RIGHT to move J4 forward, D-pad LEFT to move J4 backward.")
    print("L1 J5 forward, R1 to move J5 backward.")
    print("Select J6 forward, Strat to move J6 backward.")
    print("X J7 forward, O to move J5 backward.")

    while True:
        pygame.event.pump()  # Process joystick events

        # Read button states (Joint 3)
        button_states[J3_FWD_BUTTON] = joystick.get_button(J3_FWD_BUTTON)
        button_states[J3_REV_BUTTON] = joystick.get_button(J3_REV_BUTTON)
        button_states[J5_FWD_BUTTON] = joystick.get_button(J5_FWD_BUTTON)
        button_states[J5_REV_BUTTON] = joystick.get_button(J5_REV_BUTTON)
        button_states[J7_FWD_BUTTON] = joystick.get_button(J7_FWD_BUTTON)
        button_states[J7_REV_BUTTON] = joystick.get_button(J7_REV_BUTTON)
        button_states[J6_FWD_BUTTON] = joystick.get_button(J6_FWD_BUTTON)
        button_states[J6_REV_BUTTON] = joystick.get_button(J6_REV_BUTTON)

        # Read axis states (Joint 1)
        axis_value_fwd = joystick.get_axis(J1_FWD_AXIS)
        axis_value_rev = joystick.get_axis(J1_REV_AXIS)
        axis_states[J1_FWD_AXIS] = axis_value_fwd > DEADZONE
        axis_states[J1_REV_AXIS] = axis_value_rev > DEADZONE

        # Read D-pad (hat switch) states (Joint 2 & Joint 4)
        hat_x, hat_y = joystick.get_hat(0)  # Read the D-pad state
        hat_states[J2_FWD_HAT] = hat_y == 1  # UP on D-pad
        hat_states[J2_REV_HAT] = hat_y == -1  # DOWN on D-pad
        hat_states[J4_FWD_HAT] = hat_x == 1  # RIGHT on D-pad
        hat_states[J4_REV_HAT] = hat_x == -1  # LEFT on D-pad

        for joint, new_command in [
            ("J6", "J6_FWD" if button_states[J6_FWD_BUTTON] else "J6_REV" if button_states[J6_REV_BUTTON] else "J6_STOP"),
            ("J7", "J7_FWD" if button_states[J7_FWD_BUTTON] else "J7_REV" if button_states[J7_REV_BUTTON] else "J7_STOP"),
            ("J5", "J5_FWD" if button_states[J5_FWD_BUTTON] else "J5_REV" if button_states[J5_REV_BUTTON] else "J5_STOP"),
            ("J3", "J3_FWD" if button_states[J3_FWD_BUTTON] else "J3_REV" if button_states[J3_REV_BUTTON] else "J3_STOP"),
            ("J2", "J2_FWD" if hat_states[J2_FWD_HAT] else "J2_REV" if hat_states[J2_REV_HAT] else "J2_STOP"),
            ("J4", "J4_FWD" if hat_states[J4_FWD_HAT] else "J4_REV" if hat_states[J4_REV_HAT] else "J4_STOP"),
            ("J1", "J1_FWD" if axis_states[J1_FWD_AXIS] else "J1_REV" if axis_states[J1_REV_AXIS] else "J1_STOP"),
        ]:
            if new_command != last_sent_command[joint]:
                success = send_command_with_ack(new_command, joint)
                if success:
                    last_sent_command[joint] = new_command

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    pygame.quit()
