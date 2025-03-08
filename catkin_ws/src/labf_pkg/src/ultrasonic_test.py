import RPi.GPIO as GPIO
import time

# 設定 GPIO 引腳
TRIG_PIN = 23
ECHO_PIN = 24

# 初始化 GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    # 發送超音波脈衝
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.002)  # 確保觸發信號穩定
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 發送 10 微秒的高電平脈衝
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # 讀取 Echo 信號的時間
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    # 計算距離
    duration = end_time - start_time
    distance = (duration * 34300) / 2  # 速度是 343 m/s，換算為 cm

    return distance

try:
    while True:
        distance = measure_distance()
        print(f"測量到的距離: {distance:.2f} cm")
        time.sleep(1)  # 每秒讀取一次

except KeyboardInterrupt:
    print("測試已停止")
    GPIO.cleanup()  # 清理 GPIO 配置
