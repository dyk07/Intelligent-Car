from machine import UART
import time

# Test rear motors only. Put the car on a stand before running.

uart = UART(2, baudrate=115200, tx=16, rx=17)

MOTOR_L = "001"
MOTOR_R = "002"

MOTOR_L_FORWARD = 2000
MOTOR_L_BACK = 1000
MOTOR_R_FORWARD = 1000
MOTOR_R_BACK = 2000
MOTOR_STOP = 1500


def send_cmd(dev_id, pwm, t=0):
    cmd = "#{0}P{1:04d}T{2:04d}!".format(dev_id, int(pwm), int(t))
    print(cmd)
    uart.write(cmd.encode("utf-8"))
    time.sleep_ms(80)


def motor(left, right):
    send_cmd(MOTOR_L, left)
    send_cmd(MOTOR_R, right)


def stop():
    motor(MOTOR_STOP, MOTOR_STOP)


print("forward")
motor(MOTOR_L_FORWARD, MOTOR_R_FORWARD)
time.sleep(2)

print("back")
motor(MOTOR_L_BACK, MOTOR_R_BACK)
time.sleep(2)

print("left motor only")
motor(MOTOR_L_FORWARD, MOTOR_STOP)
time.sleep(1)

print("right motor only")
motor(MOTOR_STOP, MOTOR_R_FORWARD)
time.sleep(1)

print("stop")
stop()
