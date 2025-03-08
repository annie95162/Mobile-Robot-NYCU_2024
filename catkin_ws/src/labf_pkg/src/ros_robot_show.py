#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import threading
from std_msgs.msg import Int16MultiArray
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
####### Define Parameter #######
state = ""

# Touch sensor
sensor = [0, 0, 0]
ARotate = 8800 * 0.9
# Speed info
rotate_speed = 6
speed = 8

# Light info
max_light = 1024

# Subscribe data = [light, motorA, motorB, ultrasonic]
subscribe_data = [0, 0]
subscribe_data_queue = []
distance = 0
# Publish data
publish_queue = []
flag = True
start_state = False
rotate_state = False
lock = threading.Lock()

####### Set GPIO pins for Ultrasonic Sensor #######
TRIG_PIN = 23
ECHO_PIN = 24

####### ROS function #######
# Subscriber callback function
def callback(dataset):
    global subscribe_data
    subscribe_data_queue.append([int(dataset.data[0]), int(dataset.data[1])])

# Publisher loop function
def publisher_func():
    pub = rospy.Publisher('control', Int16MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        if publish_queue:
            msg = Int16MultiArray()
            data = publish_queue.pop(0)
            msg.data.append(data['left'])
            msg.data.append(data['right'])
            pub.publish(msg)
            rate.sleep()
        else:
            time.sleep(0.01)

####### Camera function #######
# Detect red or green color
def camera():
    global subscribe_data, publish_queue, flag, start_state
    flag = True
    cap = cv2.VideoCapture(0)  # 打開相機
    if not cap.isOpened():
        print("can't open camera")
        return
    # 設置較低的解析度和幀率來提高穩定性
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 15)
    while flag:
        # cnt = int(input())
        ret, frame = cap.read()
        if not ret:
            print("can't read camera")
            break

        # 將影像轉換為 HSV 顏色空間

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定義紅色和綠色的 HSV 範圍
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        lower_yellow = np.array([10, 100, 100])
        upper_yellow = np.array([25, 255, 255])  # 定義黃色的 HSV 範圍

        # 創建遮罩來檢測顏色
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # cv2.imshow("Camera Test", frame)
        # print("k :", k)
        # print("red now", cv2.countNonZero(red_mask))
        # print("green now", cv2.countNonZero(green_mask))
        # 判斷是否有檢測到紅色或綠色
        if cv2.countNonZero(red_mask) > 50000:  # 如果紅色像素數量大於門檻
            start_state = False
            rotate_state = True
            print("detect red and stop")
            with lock:
                publish_queue = [{'left': 0, 'right': 0}]  # 檢測到紅色停止
        elif cv2.countNonZero(green_mask) > 50000:  # 如果綠色像素數量大於門檻
            start_state = True
            rotate_state = True
            print("detect green and go")
            with lock:
                publish_queue = [{'left': speed, 'right': speed}]  # 檢測到綠色前進
                
        elif cv2.countNonZero(blue_mask) > 50000: #檢測到藍色
            print("detect blue, exiting loop")
            start_state = False
            rotate_state = False
            finish()
            flag = False
            break
               
        elif (cv2.countNonZero(yellow_mask) > 30000) and (start_state) and (rotate_state):  # 如果黃色像素數量大於門檻
            print("detect yellow, rotating 180 degrees")
            rotate_state = False
            with lock:
                publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]  # 開始旋轉
                time.sleep(3.8)  # 假設3秒可以完成180度旋轉，具體時間需要根據實際情況調整
                publish_queue = [{'left': 0, 'right': 0}]  # 停止旋轉
                time.sleep(0.2)
                print("rotation complete")
                # rotate_180()  # 執行180度迴轉
                print("and go")
                publish_queue = [{'left': speed, 'right': speed}]  # 檢測到綠色前進
           
        elif (distance <= 10) and start_state:
            rotate_state = True
            print("something back and go")
            with lock:
                publish_queue = [{'left': speed+8, 'right': speed+8}]  # 檢測到東西加速
                
        elif (distance > 10) and start_state:
            with lock:
                publish_queue = [{'left': speed, 'right': speed}]  # 沒有檢測到東西減速
                

        # 按下 ESC 鍵退出
        # time.sleep(0.2)

    cap.release()
    cv2.destroyAllWindows()
    print("finish")
    return
    # finish()
    
####### State function #######
# Go state
def go():
    print("=start go")
    global state, publish_queue
    state = "go"
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(8)
    # Monitor sensor[1] for stopping condition
    while state == "go_":
        '''
        if sensor[1] == 0:  # Left sensor detects an obstacle
            back(0.3)
            right_and_go()
            return
        '''
        if state != "go":
            print(" ")
        time.sleep(0.01)
    finish()
    return

# Back state
def back(duration):
    print("=start back")
    global state, publish_queue
    state = "back"
    publish_queue = [{'left': -speed, 'right': -speed}]
    time.sleep(duration)  # Backward for the specified duration
    
# Rotate 180 degree
def rotate_180():
    global publish_queue
    print("=start rotating 180 degrees")
    with lock:
        publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]  # 開始旋轉
    time.sleep(2.2)  # 假設3秒可以完成180度旋轉，具體時間需要根據實際情況調整
    # publish_queue = [{'left': 0, 'right': 0}]  # 停止旋轉
    print("rotation complete")
    
# Right turn and go straight towards the brightest point
def right_and_go():
    print("=start right turn and go towards brightest")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue
    state = "right_and_go"
    
    # Perform a right turn
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    # time.sleep(2.6)  # Perform right turn for a while to change direction
    time.sleep(1.9)
    print("go right done")
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight towards brightest light
    while state == "right_and_go":
        if sensor[0] == 0:
            print("go left")
            left_and_go()
        elif sensor[1] == 0:
            back(0.1)
            print("go right")
            right_and_go_again()
            return
        '''elif subscribe_data_queue:
            subscribe_data = min(subscribe_data_queue, key=lambda x: x[0])
            subscribe_data_queue = []
            # Adjust direction based on light sensor reading
            if subscribe_data[0] < max_light:
                publish_queue = [{'left': speed, 'right': speed}]
            else:
                publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]'''
        time.sleep(0.01)
        
# Right turn again and go towards the brightest point
def right_and_go_again():
    print("=start right turn and go again towards brightest!")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue
    state = "right_and_go_again"
    
    # Perform a right turn
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    time.sleep(2.4)  # Perform right turn for a while to change direction
    
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight towards brightest light
    while state == "right_and_go_again":
        if sensor[0] == 0:
            left_and_go()
        elif sensor[1] == 0:
            back(0.1)
            right_and_go_again()
            return
        '''elif subscribe_data_queue:
            subscribe_data = min(subscribe_data_queue, key=lambda x: x[0])
            subscribe_data_queue = []
            # Adjust direction based on light sensor reading
            if subscribe_data[0] < max_light:
                publish_queue = [{'left': speed, 'right': speed}]
            else:
                publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]'''
        time.sleep(0.01)

def left_and_go():
    print("=start right turn and go towards brightest")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue
    state = "left_and_go"
    
    # Perform a right turn
    publish_queue = [{'left': -(rotate_speed+5), 'right': rotate_speed+5}]
    time.sleep(0.9)  # Perform right turn for a while to change direction
    
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight towards brightest light
    while state == "left_and_go":
        if sensor[1] == 0:
            # back(0.1)
            finish()
        '''elif sensor[1] == 0:
            back(0.1)
            right_and_go_again()
            return
        elif subscribe_data_queue:
            subscribe_data = min(subscribe_data_queue, key=lambda x: x[0])
            subscribe_data_queue = []
            # Adjust direction based on light sensor reading
            if subscribe_data[0] < max_light:
                publish_queue = [{'left': speed, 'right': speed}]
            else:
                publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]'''
        time.sleep(0.01)
        
# Finish state
def finish():
    print("=start finish")
    global state, publish_queue
    publish_queue = [{'left': 0, 'right': 0}]
    state = "finish"

####### Loop #######
# Read GPIO
def read_gpio():
    print("=start read_gpio")
    global state, sensor, distance
    # Init GPIO
    # Set GPIO pin mode
    ####### GPIO Configuration #######
    '''
    GPIO.setmode(GPIO.BCM)
 
    pins = [0, 1, 2]  # GPIO0 ??GPIO2
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)

    # Save GPIO0 ~ GPIO2 to sensor[]
    while True:
        for idx, pin in enumerate(pins):
            sensor[idx] = GPIO.input(pin)
        time.sleep(0.01)
    '''
'''        
def ultra_distance():
    global distance
    while not rospy.is_shutdown():
        if subscribe_data_queue:
            # encoder_a_data = subscribe_data_queue[-1][1]  # Get the latest encoder A data
            # encoder_b_data = subscribe_data_queue[-1][2]  # Get the latest encoder B data
            # distance = subscribe_data_queue[-1][3]
            print("distance :", distance)
            # print("Encoder A value:", encoder_a_data)
            # print("Encoder B value:", encoder_b_data)
            # print("Motor value = ", encoder_a_plus_b)
        time.sleep(0.5)
'''
####### New Ultrasonic Test Function #######
def ultra_distance():
    global distance
    # GPIO configuration for Ultrasonic sensor
    GPIO.setwarnings(False)  # 關閉 GPIO 警告
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    while not rospy.is_shutdown():
        # Measure ultrasonic sensor distance
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.002)  # Ensure the trig signal is stable
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)  # Send a 10 microsecond pulse
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # Measure echo response time
        start_time = time.time()
        while GPIO.input(ECHO_PIN) == 0:
            start_time = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            end_time = time.time()

        # Calculate distance
        duration = end_time - start_time
        distance = (duration * 34300) / 2  # Speed of sound is 343 m/s, converting to cm

        # Print the measured distance (for debugging purposes)
        # print("Measured Distance: {:.2f} cm".format(distance))

        time.sleep(0.1)  # 每隔一秒進行一次測量

        
# Main function
def main():
    global state
    rospy.init_node('cp2', anonymous=True)
    
    # Start the publisher function as a thread
    threading.Thread(target=publisher_func).start()
    # threading.Thread(target=read_gpio).start()
    threading.Thread(target=ultra_distance).start()
    # threading.Thread(target=camera).start()  # 啟動相機檢測的執行緒

    # Subscriber
    rospy.Subscriber("output", Int16MultiArray, callback)

    # Get user input and add to global queue
    while not rospy.is_shutdown():
        try:
            print("1.Camera")
            print("2.Go")
            print("3.Sensor state")
            print("4.Distance")
            count = int(input("Action:"))
            if count == 1:
                threading.Thread(target=camera).start()  # 啟動相機檢測的執行緒
                #camera()
            elif count == 2:
                go()
            elif count == 3:
                print(sensor)
                print(subscribe_data)
                print(state)
            elif count == 4:
                print(distance)
                # threading.Thread(target=ultra_distance).start()
            else:
                if state != "finish":  # Check if state is not already "finish" to avoid duplicate action
                    finish()
        except ValueError:
            print("Invalid input. Please enter a number.")
            if state != "finish":
                finish()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
