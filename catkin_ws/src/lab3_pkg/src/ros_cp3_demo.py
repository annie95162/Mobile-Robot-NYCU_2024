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
rotate_speed = 4
speed = 15

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
            back(0.2)
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


# Right turn and go straight
def right_and_go():
    print("=start right turn and go")
    global state, publish_queue
    state = "right_and_go"
    
    # Perform a right turn
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    time.sleep(2.6)  # Perform right turn for a while to change direction
    
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight until left sensor detects an obstacle
    while state == "right_and_go":
        if sensor[0] == 0:
            finish()
        elif sensor[1] == 0:
            back(0.1)
            right_and_go_again()
            return
        time.sleep(0.01)
        
def right_and_go_again():
    print("=start right turn and go again!")
    global state, publish_queue
    state = "right_and_go_again"
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    time.sleep(2.4)  # Perform right turn for a while to change direction
    
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight until left sensor detects an obstacle
    while state == "right_and_go_again":
        if sensor[0] == 0:
            finish()
        elif sensor[1] == 0:
            back(0.1)
            right_and_go_again()
            return
        time.sleep(0.01)
'''
# Right turn and go straight until front sensor detects an obstacle
# Right turn and go straight until front sensor detects an obstacle
def right_and_go_until_front():
    print("=start right turn and go until front sensor detects obstacle")
    global state, publish_queue, subscribe_data, subscribe_data_queue, ARotate
    state = "right_and_go_until_front"
    
    # Perform a right turn
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    
    # Clear any old data in subscribe_data_queue
    subscribe_data_queue = []

    # Continue rotating until half circle (ARotate / 2) is completed
    while not rospy.is_shutdown():
        if subscribe_data_queue:
            # Update subscribe_data with the latest value from the queue
            subscribe_data = min(subscribe_data_queue, key=lambda x: x[0])
            subscribe_data_queue = []

            # Check if the accumulated motor counts indicate half a rotation
            if subscribe_data[1] + subscribe_data[2] >= ARotate / 4:
                print("Completed half rotation, proceed to move straight")
                
                # Stop rotating and go straight
                publish_queue = [{'left': speed, 'right': speed}]
                break

        time.sleep(0.01)
    
    time.sleep(3.0)
    publish_queue = [{'left': speed, 'right': speed}]
    # Go straight after turning right
    while state == "right_and_go_until_front":
        if sensor[0] == 0 or sensor[1] == 0:  # Front sensor detects an obstacle
            back(0.2)
            finish()
            return
        time.sleep(0.01)

'''
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
    pins = [0, 1, 2]  # GPIO0 到 GPIO2
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
        GPIO.cleanup()  # 清理 GPIO 設置，在程式結束時執行