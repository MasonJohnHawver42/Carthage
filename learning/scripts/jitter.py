import pyautogui
import time
import random

def jitter_mouse():
    # Get the current mouse position
    original_position = pyautogui.position()

    # Calculate jitter range
    jitter_range = 100

    # Jitter the mouse by moving it randomly within the range
    pyautogui.moveRel(
        x_offset:=random.randint(-jitter_range, jitter_range),
        y_offset:=random.randint(-jitter_range, jitter_range),
        duration=2
    )

    # Return the mouse to its original position
    pyautogui.moveTo(original_position, duration=0.2)

# Main loop to jitter the mouse every 30 seconds
while True:
    jitter_mouse()
    time.sleep(30)  # Wait for 30 seconds before jittering again
