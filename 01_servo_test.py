from machine import UART
import time

# Test front steering servo only. Wheels should turn left, center, right.

uart = UART(2, baudrate=115200, tx=16, rx=17)

SERVO_DIR = "003"
SERVO_CENTER = 1500
SERVO_LEFT = 1080
SERVO_RIGHT = 1920


def send_cmd(dev_id, pwm, t=0):
    cmd = "#{0}P{1:04d}T{2:04d}!".format(dev_id, int(pwm), int(t))
    print(cmd)
    uart.write(cmd.encode("utf-8"))
    time.sleep_ms(100)


while True:
    print("center")
    send_cmd(SERVO_DIR, SERVO_CENTER)
    time.sleep(1)
    print("left")
    send_cmd(SERVO_DIR, SERVO_LEFT)
    time.sleep(1)
    print("center")
    send_cmd(SERVO_DIR, SERVO_CENTER)
    time.sleep(1)
    print("right")
    send_cmd(SERVO_DIR, SERVO_RIGHT)
    time.sleep(1)
