#!/usr/bin/python2
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
ARotate = 8800 * 0.92
encoder_a_plus_b = 0
threshold = 650
# Speed info
rotate_speed = 6
speed = 15

# Light info
max_light = 1024
light_data = 1000

beacon_select = 0
beacon_lower_bound = [0.18, 0.4]
beacon_upper_bound = [0.28, 0.5]
read_ir_times = 1200 # <-set it yourself
received_ir_data = 0.0
find_ir = False
not_find_ir_threshold = 2 # <-set it yourself

# Subscribe data = [light, motorA, motorB]
subscribe_data = [0, 0, 0]
subscribe_data_queue = []

# Publish data
publish_queue = []

# lock = threading.Lock()

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

# Right turn and go straight towards the brightest point
def right_and_go():
    print("=start right turn and go towards brightest")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue, encoder_a_plus_b, light_data
    state = "right_and_go"
    
    angle_turned = 0
    target_angle = 360
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    accumulative_motor = 0
    encoder_a_plus_b = 0
    min_light_value = float('inf')
    min_light_distance = 0
    initial_encoder_value = encoder_a_plus_b
    while ARotate > encoder_a_plus_b:
        # accumulative_motor += encoder_a_plus_b
        # light_value = subscribe_data_queue[-1][0]
        print(light_data, encoder_a_plus_b)
        if light_data < min_light_value:
                min_light_value = light_data
                min_light_distance = encoder_a_plus_b - initial_encoder_value
        print("encoder: ",encoder_a_plus_b)
        time.sleep(0.1)
    
    # Stop rotation
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(0.1)
    print("Done rotating. Minimum light value found: ", min_light_value, " at distance: ", min_light_distance)
    
    # Perform a right turn
    encoder_a_plus_b = 0
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    while (min_light_distance - encoder_a_plus_b) > threshold:
        # accumulative_motor += encoder_a_plus_b
        print("encoder: ",encoder_a_plus_b)
        time.sleep(0.1)
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(0.1)
    # time.sleep(1.0)
    
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(1.0)
    publish_queue = [{'left': 0, 'right': 0}]
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
            return'''
        elif subscribe_data_queue:
            subscribe_data = min(subscribe_data_queue, key=lambda x: x[0])
            subscribe_data_queue = []
            # Adjust direction based on light sensor reading
            if subscribe_data[0] < max_light:
                publish_queue = [{'left': speed, 'right': speed}]
            else:
                publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
            # Output light value every second
            print("Light value:", subscribe_data[0])
            time.sleep(1)'''
        time.sleep(0.01)

def left_and_go():
    print("=start right turn and go towards brightest")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue, find_ir
    state = "left_and_go"
    
    # Perform a right turn
    publish_queue = [{'left': -(rotate_speed+5), 'right': rotate_speed+5}]
    time.sleep(0.8)  # Perform right turn for a while to change direction
    '''
    # find ir
    while find_ir == False:
        publish_queue = [{'left': -(rotate_speed), 'right': rotate_speed}]
    '''
    # Go straight after turning right
    publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight towards brightest light
    while state == "left_and_go":
        if sensor[1] == 0:
            finish()
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
    pins = [0, 1, 2, 3]  # GPIO0 ??GPIO2
    pins_r = [0, 1, 2]
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)

    # Save GPIO0 ~ GPIO2 to sensor[]
    while True:
        for idx, pin in enumerate(pins_r):
            sensor[idx] = GPIO.input(pin)
        time.sleep(0.01)

# Light sensor output loop
def light_sensor_output():
    global light_data
    while not rospy.is_shutdown():
        if subscribe_data_queue:
            light_data = subscribe_data_queue[-1][0]  # Get the latest light sensor value
            # print("Light sensor value:", light_data)
            # return(light_data)
        time.sleep(0.1)

# Motor A and B
def encoder_data_output_loop():
    global encoder_a_plus_b
    while not rospy.is_shutdown():
        if subscribe_data_queue:
            # encoder_a_data = subscribe_data_queue[-1][1]  # Get the latest encoder A data
            # encoder_b_data = subscribe_data_queue[-1][2]  # Get the latest encoder B data
            encoder_a_plus_b = subscribe_data_queue[-1][1] + subscribe_data_queue[-1][2]
            # print("Encoder A value:", encoder_a_data)
            # print("Encoder B value:", encoder_b_data)
            # print("Motor value = ", encoder_a_plus_b)
        time.sleep(0.01)

def ir_sensor_output():
    ir_count0 = 0.0
    ir_count1 = 0.0
    not_find_ir_counter = 0
    while not rospy.is_shutdown():
        if GPIO.input(3) == GPIO.LOW:
            ir_count0 += 1
        else:
            ir_count1 += 1

        if (ir_count0 + ir_count1) >= read_ir_times:
            received_ir_data = ir_count0 / (ir_count0 + ir_count1)
            if (received_ir_data >= beacon_lower_bound[beacon_select]) and (received_ir_data <= beacon_upper_bound[beacon_select]):
                not_find_ir_counter = 0
                find_ir = True
            else:
                not_find_ir_counter += 1
                if not_find_ir_counter >= not_find_ir_threshold:
                    find_ir = False
            ir_count0 = ir_count1 = 0.0

        time.sleep(0.0001)

# Main function
def main():
    global state, light_data
    rospy.init_node('cp2', anonymous=True)
    
    # Start the publisher function as a thread
    threading.Thread(target=publisher_func).start()
    threading.Thread(target=read_gpio).start()
    threading.Thread(target=encoder_data_output_loop).start()
    # threading.Thread(target=ir_sensor_output).start()
    threading.Thread(target=light_sensor_output).start()

    # Subscriber
    rospy.Subscriber("output", Int16MultiArray, callback)

    # Get user input and add to global queue
    while not rospy.is_shutdown():
        try:
            print("1.Go")
            print("2.Finish")
            print("3.Sensor state")
            print("4.Light sensor value")
            count = int(input("Action:"))
            if count == 1:
                if state != "go":  # Check if state is not already "go" to avoid duplicate action
                    go()
            elif count == 2:
                    finish()
            elif count == 3:
                print(sensor)
                print(subscribe_data)
                print(state)
            elif count == 4:
                # threading.Thread(target=light_sensor_output).start()
                print(light_data)
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
