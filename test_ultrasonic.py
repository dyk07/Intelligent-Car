from machine import Pin, time_pulse_us
import time

# 请根据实际接线修改引脚
TRIG = Pin(41, Pin.OUT)
ECHO = Pin(1, Pin.IN)

def get_distance():
    """返回距离（单位：cm），若超时则返回-1"""
    TRIG.value(0)
    time.sleep_us(2)
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)
    
    pulse_us = time_pulse_us(ECHO, 1, 30000)  # 超时30ms
    if pulse_us < 0:
        return -1
    distance = pulse_us / 58.0  # 声速340m/s，往返
    return distance

if __name__ == "__main__":
    print("开始测试超声波雷达...")
    while True:
        dist = get_distance()
        if dist < 0:
            print("测量超时")
        else:
            print("距离: {:.2f} cm".format(dist))
        time.sleep(0.3)