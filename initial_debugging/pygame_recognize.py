import pygame

# Initialize pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect one and restart.")
    pygame.quit()
    exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Deadzone threshold (for triggers)
DEADZONE = 0.2

# Button mapping (PlayStation-style controllers)
button_map = {
    0: "0 (X)",
    1: "1 (O)",
    2: "2 (Square)",
    3: "3 (Triangle)",
    4: "4 (L1)",
    5: "5 (R1)",
    6: "6 (Select)",
    7: "7 (Start)",
}

# Axis mapping for triggers
axis_map = {
    4: "4 (L2)",  # L2 is an axis
    5: "5 (R2)",  # R2 is an axis
}

# D-pad (Hat Switch) button numbers
dpad_map = {
    (0, 1): "10 (D-pad Up)",
    (0, -1): "11 (D-pad Down)",
    (-1, 0): "12 (D-pad Left)",
    (1, 0): "13 (D-pad Right)",
}

# Main loop
running = True
current_input = ""

while running:
    pygame.event.pump()  # Process events

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Button Press
        elif event.type == pygame.JOYBUTTONDOWN:
            button_name = button_map.get(event.button, f"{event.button}")
            current_input = f"Button {button_name} Pressed"

        # Button Release
        elif event.type == pygame.JOYBUTTONUP:
            current_input = ""

        # Axis Movement (L2/R2 Triggers)
        elif event.type == pygame.JOYAXISMOTION:
            axis_value = joystick.get_axis(event.axis)

            # Apply Deadzone for triggers
            if abs(axis_value) > DEADZONE:
                axis_name = axis_map.get(event.axis, f"Axis {event.axis}")
                current_input = f"Axis {axis_name}: {axis_value:.2f}"
            else:
                current_input = ""

        # D-pad (Hat Switch) as Buttons
        elif event.type == pygame.JOYHATMOTION:
            hat_value = joystick.get_hat(0)
            if hat_value in dpad_map:
                current_input = f"Button {dpad_map[hat_value]} Pressed"
            else:
                current_input = ""

    # Clear console and print current input
    if current_input:
        print("\033c", end="")  # Clear console
        print(current_input)

# Quit pygame
pygame.quit()
