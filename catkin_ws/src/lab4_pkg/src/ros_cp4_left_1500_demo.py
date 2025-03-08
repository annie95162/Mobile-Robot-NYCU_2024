#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import threading
from std_msgs.msg import Int16MultiArray
import time
import RPi.GPIO as GPIO

####### Define Parameter #######
state = ""

# Touch sensor
sensor = [0, 0, 0]
ARotate = 8800 * 0.9
# Speed info
rotate_speed = 6
speed = 15

# Light info
max_light = 1024

# Subscribe data = [light, motorA, motorB]
subscribe_data = [0, 0, 0]
subscribe_data_queue = []

# Publish data
publish_queue = []

lock = threading.Lock()

####### ROS function #######
# Subscriber callback function
def callback(dataset):
    global subscribe_data
    subscribe_data_queue.append([int(dataset.data[0]), int(dataset.data[1]), int(dataset.data[2])])

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

####### State function #######
# Go state
def go():
    print("=start go")
    global state, publish_queue
    state = "go"
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Monitor sensor[1] for stopping condition
    while state == "go":
        if sensor[1] == 0:  # Left sensor detects an obstacle
            back(0.3)
            right_and_go()
            return
        time.sleep(0.01)

# Back state
def back(duration):
    print("=start back")
    global state, publish_queue
    state = "back"
    publish_queue = [{'left': -speed, 'right': -speed}]
    time.sleep(duration)  # Backward for the specified duration

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
    global state, sensor
    # Init GPIO
    GPIO.setmode(GPIO.BCM)
    # Set GPIO pin mode
    pins = [0, 1, 2]  # GPIO0 ??GPIO2
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)

    # Save GPIO0 ~ GPIO2 to sensor[]
    while True:
        for idx, pin in enumerate(pins):
            sensor[idx] = GPIO.input(pin)
        time.sleep(0.01)

# Main function
def main():
    global state
    rospy.init_node('cp2', anonymous=True)
    
    # Start the publisher function as a thread
    threading.Thread(target=publisher_func).start()
    threading.Thread(target=read_gpio).start()

    # Subscriber
    rospy.Subscriber("output", Int16MultiArray, callback)

    # Get user input and add to global queue
    while not rospy.is_shutdown():
        try:
            print("1.Go")
            print("2.Finish")
            print("3.Sensor state")
            count = int(input("Action:"))
            if count == 1:
                if state != "go":  # Check if state is not already "go" to avoid duplicate action
                    go()
            elif count == 3:
                print(sensor)
                print(subscribe_data)
                print(state)
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
