from machine import UART, Pin, time_pulse_us
import time

# ===== Hardware Configuration =====
uart = UART(2, baudrate=115200, tx=16, rx=17)

MOTOR_L = "001"
MOTOR_R = "002"
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
SERVO_RIGHT  = 2100

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
    
    # Drive forward slightly to clear the current cross completely
    set_servo(SERVO_CENTER)
    motor(FORWARD_L, FORWARD_R)
    time.sleep_ms(350) 

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
    HIGH_DENSITY_WINDOW_MS = 100 
    
    last_intersection_time = 0
    IMMUNITY_DURATION_MS = 1000  
    KEEP_ERROR_THRESHOLD = 350
    should_lock_motion = False    
    
    saved_servo_pwm = SERVO_CENTER
    saved_motor_l = MOTOR_STOP
    saved_motor_r = MOTOR_STOP
    
    cnt = 0
    loop_counter = 0

    current_servo = SERVO_CENTER
    current_motor_l = FORWARD_L
    current_motor_r = FORWARD_R
    
    ERROR_WINDOW_SIZE = 4  # Number of past frames to average (try 3 to 7)
    error_history = []
    ACT_WINDOW_SIZE = 4
    active_num = []

    while True:
        loop_counter += 1
        current_time = time.ticks_ms()
        
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
        
        if vals[0] == 1:
            last_left_wing_time = current_time
        if vals[4] == 1:
            last_right_wing_time = current_time

        if sum(active_num)/len(active_num) > 3.5:
            last_high_density_time = current_time

        time_since_last_cross = time.ticks_diff(current_time, last_intersection_time)
        
        time_diff = abs(time.ticks_diff(last_left_wing_time, last_right_wing_time))
        left_hit_recently = (time.ticks_diff(current_time, last_left_wing_time) < CROSS_WINDOW_MS)
        right_hit_recently = (time.ticks_diff(current_time, last_right_wing_time) < CROSS_WINDOW_MS)
        broad_hit_recently = (time.ticks_diff(current_time, last_high_density_time) < HIGH_DENSITY_WINDOW_MS)

        # Intersection processing
        if time_since_last_cross > IMMUNITY_DURATION_MS:
            if (left_hit_recently and right_hit_recently and time_diff < CROSS_WINDOW_MS) or broad_hit_recently:
                if not in_intersection:
                    saved_servo_pwm = current_servo
                    saved_motor_l = current_motor_l
                    saved_motor_r = current_motor_r
                    
                    if abs(sum(active_num) / len(active_num)) >= KEEP_ERROR_THRESHOLD:
                        should_lock_motion = True
                        print("=== SWERVING DETECTED: Locking motion state ===")
                    else:
                        should_lock_motion = False
                        print("=== RUNNING STRAIGHT: Normal tracking active ===")
                    
                    handle_intersection()
                    in_intersection = True
                    
                    last_intersection_time = time.ticks_ms() 
                    last_left_wing_time = 0
                    last_right_wing_time = 0
                    last_high_density_time = 0
                    cnt = 0 
                continue
            else:
                if sum(active_num)/len(active_num) <= 2 and vals[0] == 0 and vals[4] == 0:
                    in_intersection = False
        else:
            in_intersection = False

        # --- EXECUTION ENGINE ---
        if time_since_last_cross <= IMMUNITY_DURATION_MS and should_lock_motion:
            set_servo(saved_servo_pwm)
            motor(saved_motor_l, saved_motor_r)
            current_servo = saved_servo_pwm
            current_motor_l = saved_motor_l
            current_motor_r = saved_motor_r
        else:
            """if CROSS_CNT >= 7:
                stop()
                break"""
                
            if sum(active_num) / len(active_num) >= 1:
                print(sum(active_num) / len(active_num), "tracking")
                cnt = 0
                weights = [-2, -1, 0, 1, 2]
                raw_error = sum(v * w for v, w in zip(vals, weights)) / (sum(active_num) / len(active_num))
                
                # Update the moving average buffer
                error_history.append(raw_error)
                if len(error_history) > ERROR_WINDOW_SIZE:
                    error_history.pop(0)
                
                # Calculate the smoothed error
                avg_error = sum(error_history) / len(error_history)
                
                # Use avg_error for steering calculation
                current_servo = SERVO_CENTER - int(avg_error * 280)
                set_servo(current_servo)
                
                # Use avg_error for motor turning decisions
                if avg_error < -0.4:
                    current_motor_l = TURN_SPEED_L
                    current_motor_r = FORWARD_R
                    last_direction = -1
                elif avg_error > 0.4:
                    current_motor_l = FORWARD_L
                    current_motor_r = TURN_SPEED_R
                    last_direction = 1
                else:
                    current_motor_l = FORWARD_L
                    current_motor_r = FORWARD_R
                    if time_since_last_cross > IMMUNITY_DURATION_MS:
                        last_direction = 0
                    
                motor(current_motor_l, current_motor_r)
                
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
                
                if cnt >= 4:
                    print(sum(active_num) / len(active_num), "forcing right")
                    current_servo = SERVO_RIGHT + 100
                    current_motor_l = 2000
                    current_motor_r = 1300
                else:   
                    if last_direction == -1:
                        print(sum(active_num) / len(active_num), "trying right")
                        current_servo = SERVO_RIGHT - 100
                        current_motor_l = FORWARD_L
                        current_motor_r = TURN_SPEED_R
                        cnt += 1
                    elif last_direction == 1:
                        print(sum(active_num) / len(active_num), "trying left")
                        current_servo = SERVO_LEFT + 100
                        current_motor_l = FORWARD_L
                        current_motor_r = TURN_SPEED_R
                        cnt += 1
                    
                    
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