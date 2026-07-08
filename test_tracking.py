from machine import Pin
import time

# 请根据实际接线修改引脚号（从左到右依次为 OUT1~OUT5）
TRACK_PINS = [47, 48, 39, 40, 4]
track_sensors = [Pin(p, Pin.IN) for p in TRACK_PINS]

def read_track():
    """返回长度为5的列表，0=白线，1=黑线（依据PDF说明）"""
    return [s.value() for s in track_sensors]

def print_track_status():
    vals = read_track()
    print("Track sensors (L->R):", vals)
    # 简单判别：全0为直道，左侧黑则左转，右侧黑则右转
    left = vals[0] + vals[1]
    right = vals[3] + vals[4]
    if left > right:
        print("-> 偏左")
    elif right > left:
        print("-> 偏右") 
    else: 
        if vals[2] == 1:
            print("-> 居中黑线")
        else:
            print("-> 无黑线或偏离")

if __name__ == "__main__":
    print("开始测试五路循迹传感器...")
    while True:
        print_track_status()
        time.sleep(0.5)