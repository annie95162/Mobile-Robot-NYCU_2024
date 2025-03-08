#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import threading
from std_msgs.msg import Int16MultiArray
import time
import RPi.GPIO as GPIO

####### Define Parameter #######
# Light sensor
sensor = [0, 0, 0]

lock = threading.Lock()

####### ROS function #######
# Subscriber callback function
def callback(dataset):
    global sensor
    sensor = [int(dataset.data[0]), int(dataset.data[1]), int(dataset.data[2])]

# Publisher loop function
def publisher_func():
    pub = rospy.Publisher('light_readings', Int16MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        with lock:
            msg.data = sensor
        pub.publish(msg)
        rate.sleep()

####### Loop #######
# Read GPIO
def read_gpio():
    print("=start read_gpio")
    global sensor
    # Init GPIO
    GPIO.setmode(GPIO.BCM)
    # Set GPIO pin mode
    pins = [0, 1, 2]  # GPIO0, GPIO1, GPIO2
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)

    # Save GPIO0 ~ GPIO2 to sensor[]
    while True:
        with lock:
            for idx, pin in enumerate(pins):
                sensor[idx] = GPIO.input(pin)
        time.sleep(0.01)

# Main function
def main():
    rospy.init_node('light_sensor_node', anonymous=True)
    
    # Start the publisher function as a thread
    threading.Thread(target=publisher_func).start()
    threading.Thread(target=read_gpio).start()

    # Subscriber
    rospy.Subscriber("output", Int16MultiArray, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
