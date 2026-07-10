#track only
from machine import UART, Pin, time_pulse_us
import time

# ===== Hardware Configuration =====
uart = UART(2, baudrate=115200, tx=16, rx=17)

MOTOR_L = "002"
MOTOR_R = "001"
SERVO_DIR = "003"

# Speed Calibration
MOTOR_STOP = 1500
FORWARD_L = 2000
BACKWARD_L = 1200
FORWARD_R = 1000
BACKWARD_R = 1800
# Right motor is physically reversed
TURN_SPEED_L = 1800
TURN_SPEED_R = 1200

# Servo Calibration
SERVO_CENTER = 1500
SERVO_LEFT   = 1000
SERVO_RIGHT  = 2200

# Track Sensors (L0, L1, Center, R1, R2)
TRACK_PINS = [47, 48, 39, 40, 4]
track = [Pin(p, Pin.IN) for p in TRACK_PINS]

# Ultrasonic Pins
TRIG = Pin(41, Pin.OUT)
ECHO = Pin(1, Pin.IN)

CROSS_CNT = 0

# ===== Communication & Core Actions =====
def send_cmd(dev_id, pwm, t=0):
    cmd = "#{0}P{1:04d}T{2:04d}!".format(dev_id, int(pwm), int(t))
    uart.write(cmd.encode("utf-8"))
    time.sleep_ms(20) # Smooth communication window

def motor(left, right):
    send_cmd(MOTOR_L, left)
    send_cmd(MOTOR_R, right)

def set_servo(pwm):
    # Bound-check safety clamp to protect steering gears
    pwm = max(SERVO_LEFT, min(SERVO_RIGHT, int(pwm)))
    send_cmd(SERVO_DIR, pwm)

def stop():
    motor(MOTOR_STOP, MOTOR_STOP)

# ===== Sensor Processing Functions =====
def read_track():
    # Returns 1 for black line, 0 for white space
    return [s.value() for s in track]

def get_distance_raw():
    TRIG.value(0)
    time.sleep_us(2)
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)
    pulse = time_pulse_us(ECHO, 1, 30000)
    if pulse < 0:
        return 999
    return pulse / 58.0

def get_distance():
    """Takes a median of 3 samples to ignore sensor anomalies."""
    readings = []
    for _ in range(3):
        d = get_distance_raw()
        readings.append(d)
        time.sleep_ms(2)
    readings.sort()
    return readings[1]

# ===== Specialized Maneuvers =====
def handle_intersection():
    """Stops perfectly over the intersection for the mandatory 1.5s rule."""
    print("=== INTERSECTION DETECTED: Pausing for 1.5 seconds ===")
    global CROSS_CNT
    CROSS_CNT += 1
    stop()
    time.sleep(1.5)  # Exact rules compliance

def bypass_obstacle():
    """Bypasses A4 boards safely and targets re-acquiring the line."""
    print("=== OBSTACLE IN PATH: Commencing Maneuver ===")
    
    # Step 1: Arc Left around the obstacle
    set_servo(SERVO_LEFT)
    motor(FORWARD_L, FORWARD_R)
    time.sleep_ms(700)
    
    # Step 2: Straighten past it
    set_servo(SERVO_CENTER)
    time.sleep_ms(600)
    
    # Step 3: Arc Right back toward the path until line is detected
    set_servo(SERVO_RIGHT)
    timeout = time.ticks_add(time.ticks_ms(), 3500)
    
    while time.ticks_diff(timeout, time.ticks_ms()) > 0:
        if sum(read_track()) > 0:
            print("Line re-acquired successfully!")
            break
        time.sleep_ms(10)
        
    set_servo(SERVO_CENTER)

# ===== Main Executive Loop =====
def run():
    print("System armed. Starting execution...")
    set_servo(SERVO_CENTER)
    time.sleep(0.5)
    
    last_direction = 0   
    in_intersection = False
    obstacle_threshold = 20.0 
    
    last_left_wing_time = 0
    last_right_wing_time = 0
    CROSS_WINDOW_MS = 60
    
    last_high_density_time = 0
    HIGH_DENSITY_WINDOW_MS = 80
    
    last_intersection_time = 0
    IMMUNITY_DURATION_MS = 1000  
    
    # --- Turning Mode Variables ---
    turning = False
    has_turned = False              # Interlock flag to enforce a single execution limit
    is_turning_right = False
    right_turn_start_time = 0
    locked_servo = SERVO_RIGHT
    locked_motor_l = 2000
    locked_motor_r = 1700
    waiting_to_cancel_turning = False
    intersection_cleared_time = 0
    
    cnt = 0
    loop_counter = 0

    current_servo = SERVO_CENTER
    current_motor_l = FORWARD_L
    current_motor_r = FORWARD_R
    
    ERROR_WINDOW_SIZE = 2  # Number of past frames to average (try 3 to 7)
    error_history = []
    ACT_WINDOW_SIZE = 2
    active_num = []

    while True:
        loop_counter += 1
        current_time = time.ticks_ms()
        
        # Check if we need to exit turning mode after 800ms from intersection restart
        if waiting_to_cancel_turning and time.ticks_diff(current_time, intersection_cleared_time) > 2500:
            turning = False
            waiting_to_cancel_turning = False
            is_turning_right = False
            print("=== TURNING MODE: Cancelled, resuming normal tracking ===")

        # Ultrasonic pacing
        """if loop_counter % 4 == 0:
            if get_distance() < obstacle_threshold:
                stop()
                time.sleep_ms(200)
                bypass_obstacle()
                continue"""

        vals = read_track()
        active_num.append(sum(vals))
        if len(active_num) > ACT_WINDOW_SIZE:
            active_num.pop(0)
            
        active_density = sum(active_num) / len(active_num) if len(active_num) > 0 else 0
        
        if vals[0] == 1:
            last_left_wing_time = current_time
        if vals[4] == 1:
            last_right_wing_time = current_time

        if active_density >= 2.5:
            last_high_density_time = current_time

        time_since_last_cross = time.ticks_diff(current_time, last_intersection_time)
        
        time_diff = abs(time.ticks_diff(last_left_wing_time, last_right_wing_time))
        left_hit_recently = (time.ticks_diff(current_time, last_left_wing_time) < CROSS_WINDOW_MS)
        right_hit_recently = (time.ticks_diff(current_time, last_right_wing_time) < CROSS_WINDOW_MS)
        broad_hit_recently = (time.ticks_diff(current_time, last_high_density_time) < HIGH_DENSITY_WINDOW_MS)

        # Intersection processing
        if turning and not waiting_to_cancel_turning:
            if active_density >= 2.5:
                print("=== TURNING MODE: Intersection detected (density >= 3) ===")
                handle_intersection()
                in_intersection = True
                intersection_cleared_time = time.ticks_ms()
                waiting_to_cancel_turning = True
                
                last_intersection_time = time.ticks_ms() 
                last_left_wing_time = 0
                last_right_wing_time = 0
                last_high_density_time = 0
                cnt = 0 
                continue
        elif time_since_last_cross > IMMUNITY_DURATION_MS:
            if (left_hit_recently and right_hit_recently and time_diff < CROSS_WINDOW_MS) or broad_hit_recently:
                if not in_intersection:
                    print("=== INTERSECTION DETECTED: Pausing for 1.5 seconds ===")
                    
                    handle_intersection()
                    in_intersection = True
                    
                    last_intersection_time = time.ticks_ms() 
                    last_left_wing_time = 0
                    last_right_wing_time = 0
                    last_high_density_time = 0
                    cnt = 0 
                continue
            else:
                if active_density <= 2 and vals[0] == 0 and vals[4] == 0:
                    in_intersection = False
        else:
            in_intersection = False

        # --- EXECUTION ENGINE ---
        if turning:
            # Execute the forced continuous right turn
            set_servo(locked_servo)
            motor(locked_motor_l, locked_motor_r)
            current_servo = locked_servo
            current_motor_l = locked_motor_l
            current_motor_r = locked_motor_r
        else:
            """if CROSS_CNT >= 7:
                stop()
                break"""
                
            current_is_turning_right = False
                
            if active_density >= 1:
                print(active_density, "tracking")
                cnt = 0
                weights = [-2, -1, 0, 1.2, 2]
                raw_error = sum(v * w for v, w in zip(vals, weights)) / active_density
                
                # Update the moving average buffer
                error_history.append(raw_error)
                if len(error_history) > ERROR_WINDOW_SIZE:
                    error_history.pop(0)
                
                # Calculate the smoothed error
                avg_error = sum(error_history) / len(error_history)
                
                # Use avg_error for steering calculation
                current_servo = SERVO_CENTER - int(avg_error * 250)
                
                # Use avg_error for motor turning decisions
                if avg_error < -0.4:
                    current_motor_l = TURN_SPEED_L
                    current_motor_r = FORWARD_R
                    last_direction = -1
                elif avg_error > 0.4:
                    current_motor_l = FORWARD_L
                    current_motor_r = TURN_SPEED_R
                    last_direction = 1
                    current_is_turning_right = True # Mark as turning right
                else:
                    current_motor_l = FORWARD_L
                    current_motor_r = FORWARD_R
                    if time_since_last_cross > IMMUNITY_DURATION_MS:
                        last_direction = 0
                    
            else:
                if len(error_history) > 0:
                    history_mean = sum(error_history) / len(error_history)
                    
                    # Set last_direction based on the stable history average
                    if history_mean > 0.1:     # Drifted right, need to recover right
                        last_direction = 1
                    elif history_mean < -0.1:  # Drifted left, need to recover left
                        last_direction = -1
                    # If it's close to 0, keep the previous last_direction value
                    
                    # Clear the history now that we've extracted its directional value
                    error_history.clear()
                
                if cnt >= 20 :
                    print(active_density, "forcing right")
                    current_servo = SERVO_RIGHT + 100
                    current_motor_l = 2000
                    current_motor_r = 1300
                    current_is_turning_right = True # Mark as forcing right
                else:   
                    if last_direction == -1:
                        print(active_density, "trying right")
                        current_servo = SERVO_RIGHT - 100
                        current_motor_l = FORWARD_L
                        current_motor_r = TURN_SPEED_R
                        cnt += 1
                        current_is_turning_right = True # Mark as trying right
                    elif last_direction == 1:
                        print(active_density, "trying left")
                        current_servo = SERVO_LEFT + 100
                        current_motor_l = FORWARD_L
                        current_motor_r = TURN_SPEED_R
                        cnt += 1
            
            # --- Continuous Right Turn Timer Update ---
            # Added "not has_turned" condition to lock out future entries
            if current_is_turning_right and not has_turned:
                if not is_turning_right:
                    is_turning_right = True
                    right_turn_start_time = current_time
                # Check if it has been turning right for over 1 second (500 ms constraint preserved from your file)
                elif time.ticks_diff(current_time, right_turn_start_time) > 500 and not turning and CROSS_CNT >= 3:
                    turning = True
                    has_turned = True # Set flag so it can never be reactivated later
                    # Force a continuous right turn instead of freezing current values
                    locked_servo = SERVO_RIGHT + 100
                    locked_motor_l = FORWARD_L
                    locked_motor_r = TURN_SPEED_R
                    print("=== TURNING MODE: Activated! Forcing continuous right turn (Single-use ONLY) ===")
            else:
                is_turning_right = False
                    
            set_servo(current_servo)
            motor(current_motor_l, current_motor_r)

        time.sleep_ms(10)

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        stop()
        set_servo(SERVO_CENTER)
        print("Program halted safely.")