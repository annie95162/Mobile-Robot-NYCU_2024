import RPi.GPIO as GPIO
import time

# �]�w GPIO �޸}
TRIG_PIN = 23
ECHO_PIN = 24

# ��l�� GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    # �o�e�W���i�߽�
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.002)  # �T�OĲ�o�H��í�w
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # �o�e 10 �L�����q���߽�
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Ū�� Echo �H�����ɶ�
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    # �p��Z��
    duration = end_time - start_time
    distance = (duration * 34300) / 2  # �t�׬O 343 m/s�A���⬰ cm

    return distance

try:
    while True:
        distance = measure_distance()
        print(f"���q�쪺�Z��: {distance:.2f} cm")
        time.sleep(1)  # �C��Ū���@��

except KeyboardInterrupt:
    print("���դw����")
    GPIO.cleanup()  # �M�z GPIO �t�m
