import time
from machine import Pin
import run_B as run # choose from [run_A, run_B] 

# Configure the BOOT button (GPIO 0)
boot_button = Pin(0, Pin.IN, Pin.PULL_UP)

print("\n--- ESP32-S3 BOOT MANAGER ---")

# --- NEW: TIMING CAPTURE BUFFER ---
# Give the user a tiny 200ms window to be caught holding the button
button_pressed = False
start_check = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start_check) < 200:
    if boot_button.value() == 0:
        button_pressed = True
        break
    time.sleep_ms(5)
# ----------------------------------

# Read the current persistent state from disk
current_mode = "RUN"
try:
    with open("mode.txt", "r") as f:
        current_mode = f.read().strip()
except OSError:
    with open("mode.txt", "w") as f:
        f.write("RUN")

# State Logic Machine
if button_pressed:
    if current_mode == "RUN":
        current_mode = "SAFE"
        print(">>> BOOT button captured! Switching to SAFE MODE.")
    else:
        current_mode = "RUN"
        print(">>> BOOT button captured! Switching to RUN MODE.")
        
    with open("mode.txt", "w") as f:
        f.write(current_mode)
    
    print("Release the button...")
    # Wait until you completely let go of the button so it doesn't double-trigger
    while boot_button.value() == 0:
        time.sleep_ms(10)
    time.sleep_ms(500)

# Execute behavior based on the mode
if current_mode == "SAFE":
    print("=============================================")
    print("STATUS: [SAFE MODE ACTIVE]")
    print("Motors disabled. Thonny can connect safely.")
    print("To arm the car: Hold BOOT and press RESET.")
    print("=============================================")
    run.stop()
    run.set_servo(run.SERVO_CENTER)

else:
    print("=============================================")
    print("STATUS: [RUN MODE ACTIVE]")
    print("Launching line tracking in 2 seconds...")
    print("To abort/stop: Hold BOOT and press RESET.")
    print("=============================================")
    time.sleep(2.0) 
    
    try:
        run.run()
    except KeyboardInterrupt:
        run.stop()
        run.set_servo(run.SERVO_CENTER)
        print("Execution interrupted manually.")