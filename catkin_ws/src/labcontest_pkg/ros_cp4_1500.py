#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
import threading
from std_msgs.msg import Int16MultiArray
import time
import RPi.GPIO as GPIO

####### Define Parameter #######
state = ""
s_state = "not start"

# Touch sensor
sensor = [0, 0, 0]
ARotate = 8800 * 0.92
encoder_a_plus_b = 0
threshold = 850
# Speed info
rotate_speed = 8
rotate_speed_r = 6
speed = 20
speed_r = 22
motor_not_move = False
not_move_threshold = 1000
motor_not_move_time = 0
# Light info
max_light = 1024
light_data = 1000

turn_right = 2350
turn_left = 2300
turn_left_C = 200
turn_right_C = 800
beacon_select = 1
beacon_select_r = 0
# 600: 0.24
# 1500: 0.454
beacon_lower_bound = [0.18, 0.4]
beacon_upper_bound = [0.28, 0.5]
read_ir_times = 1200 # <-set it yourself
received_ir_data = 0.0
find_ir = False
find_r_ir = False
not_find_ir = False
not_find_ir_counter = 0
not_find_ir_threshold = 50 # <-set it yourself
time_for_search_light = 2.5
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
def go(): #旋轉後直走
    print("=start go")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue, encoder_a_plus_b, light_data, time_for_search_light, find_ir
    state = "go"
    # publish_queue = [{'left': speed, 'right': speed}]
    angle_turned = 0
    target_angle = 360
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    encoder_a_plus_b = 0
    min_light_value = float('inf')
    min_light_distance = 0
    initial_encoder_value = encoder_a_plus_b
    while ARotate > encoder_a_plus_b:
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
    # time.sleep(1.0)
    # publish_queue = [{'left': 0, 'right': 0}]
    # Continue moving straight towards brightest light
    no_light_detected_timer = time.time()
    while state == "go":
        if (time.time() - no_light_detected_timer) > time_for_search_light:
            print("go find light again")
            right_and_go()
            return
        elif sensor[0] == 0:
            print("find ir")
            find_ir = False
            left_and_go()
            return
        elif sensor[1] == 0:
            back(0.2)
            print("collision find light again")
            right_and_go()
            return
        time.sleep(0.01)
    # Monitor sensor[1] for stopping condition
    '''while state == "go":
        if sensor[1] == 0:  # Left sensor detects an obstacle
            back(0.2)
            right_and_go()
            return
        time.sleep(0.01)'''

# Back state
def back(duration):
    print("=start back")
    global state, publish_queue
    state = "back"
    publish_queue = [{'left': -speed, 'right': -speed}]
    time.sleep(duration)  # Backward for the specified duration

# Right turn and go straight towards the brightest point
def right_and_go():
    print("=start again not found light yet")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue, encoder_a_plus_b, light_data, time_for_search_light, find_ir
    state = "right_and_go"
    
    angle_turned = 0
    target_angle = 360
    publish_queue = [{'left': rotate_speed, 'right': -rotate_speed}]
    encoder_a_plus_b = 0
    min_light_value = float('inf')
    min_light_distance = 0
    initial_encoder_value = encoder_a_plus_b
    while ARotate > encoder_a_plus_b:
        # accumulative_motor += encoder_a_plus_b
        # light_value = subscribe_data_queue[-1][0]
        # print(light_data, encoder_a_plus_b)
        if light_data < min_light_value:
                min_light_value = light_data
                min_light_distance = encoder_a_plus_b - initial_encoder_value
        # print("encoder: ",encoder_a_plus_b)
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
    # time.sleep(1.0)
    # publish_queue = [{'left': 0, 'right': 0}]
    # Continue moving straight towards brightest light
    no_light_detected_timer = time.time()
    while state == "right_and_go":
        if (time.time() - no_light_detected_timer) > time_for_search_light:
            print("go find light again")
            right_and_go()
            return
        elif sensor[0] == 0:
            print("find ir")
            find_ir = False
            left_and_go()
            return
        elif sensor[1] == 0:
            back(0.2)
            print("collision find light again")
            right_and_go()
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
    print("=start find ir and already find light")
    global state, publish_queue, max_light, subscribe_data, subscribe_data_queue, find_ir, received_ir_data, find_r_ir, encoder_a_plus_b, light_data, not_find_ir, not_find_ir_counter, s_state, motor_not_move_time
    state = "left_and_go"
    s_state = "start"
    flag = False
    # Perform a right turn
    # publish_queue = [{'left': -rotate_speed, 'right': rotate_speed}]
    # time.sleep(0.8)  # Perform right turn for a while to change direction
    find_ir = False
    find_r_ir = False
    not_find_ir = False
    not_find_ir_counter = 0
    max_ir = 0
    max_ir_distance = 3000
    # find ir
    while flag == False:
        publish_queue = [{'left': -(rotate_speed_r), 'right': rotate_speed_r}]
        if (sensor[1] == 0) and (received_ir_data > 0.1): #進到洞裡
            print("In the hole yay")
            flag = True
            finish()
            return
        elif (sensor[1] == 0) and (received_ir_data <= 0.1): #找到但沒進到洞裡
            back(0.2)
            print("Not in the hole yet.")
        elif light_data <= 150: #結束程式
            flag = True
            finish()
            return
        elif not_find_ir == True: #一段時間找不到ir
            print("Not find ir and go")
            motor_not_move_time = 0
            encoder_a_plus_b = 0
            while ARotate > encoder_a_plus_b: 
                if received_ir_data > max_ir:
                    max_ir = received_ir_data
                    max_ir_distance = encoder_a_plus_b
                # print("encoder: ",encoder_a_plus_b)   
                time.sleep(0.1)
            encoder_a_plus_b = 0
            while (max_ir_distance - encoder_a_plus_b) > threshold:
                # accumulative_motor += encoder_a_plus_b
                time.sleep(0.1)
            publish_queue = [{'left': speed, 'right': speed}]
            time.sleep(2.0)
            publish_queue = [{'left': -(speed-1), 'right': -(speed)}]
            time.sleep(0.2)
            
            not_find_ir = False
            not_find_ir_counter = 0
        elif find_ir == True: #找到ir就往方向走
            print("Find ir")
            publish_queue = [{'left': speed, 'right': speed}]
            time.sleep(1.5)
            find_ir = False
        time.sleep(0.0001)
    # publish_queue = [{'left': -rotate_speed, 'right': rotate_speed}]
    # Go straight after turning right
    # publish_queue = [{'left': speed, 'right': speed}]
    
    # Continue moving straight towards ir
    '''
    while state == "left_and_go":
        if sensor[1] == 0:
            finish()
        time.sleep(0.01)
    '''    
# Finish state
def finish():
    print("=start finish")
    global state, publish_queue, s_state
    publish_queue = [{'left': 0, 'right': 0}]
    state = "finish"
    s_state = "not start"
    

# Start from A
def start_from_A():
    print("=start from A")
    global encoder_a_plus_b, publish_queue
    encoder_a_plus_b = 0
    publish_queue = [{'left': rotate_speed, 'right': -(rotate_speed)}]
    while turn_right > encoder_a_plus_b:    
        time.sleep(0.1)
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(1.8)
    encoder_a_plus_b = 0
    publish_queue = [{'left': -(rotate_speed), 'right': rotate_speed}]
    while turn_left > encoder_a_plus_b:    
        time.sleep(0.1)
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(2.7)
    if sensor[0] == 1:
        go()
        return
    else:
        encoder_a_plus_b = 0
        publish_queue = [{'left': -(rotate_speed), 'right': rotate_speed}]
        while turn_left > encoder_a_plus_b:    
            time.sleep(0.1)
        publish_queue = [{'left': speed, 'right': speed}]
        time.sleep(3.5)
        if(received_ir_data <= 0.1):
            left_and_go()
            return
        else:
            finish()
            return
    
# Start from B
def start_from_B():
    print("=start from B")
    global encoder_a_plus_b, publish_queue
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(2.7)
    if sensor[0] == 1:
        go()
        return
    else:
        publish_queue = [{'left': speed, 'right': speed}]
        time.sleep(3.5)
        if(received_ir_data <= 0.1):
            left_and_go()
            return
        else:
            finish()
            return
    
# Start from C
def start_from_C():
    print("=start from C")
    global encoder_a_plus_b, publish_queue
    encoder_a_plus_b = 0
    publish_queue = [{'left': -(rotate_speed), 'right': rotate_speed}]
    while turn_left_C > encoder_a_plus_b:    
        time.sleep(0.1)
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(2.7)
    if sensor[0] == 1:
        go()
        return
    else:    
        encoder_a_plus_b = 0
        publish_queue = [{'left': rotate_speed, 'right': -(rotate_speed)}]
        while turn_right_C > encoder_a_plus_b:    
            time.sleep(0.1)
        publish_queue = [{'left': speed, 'right': speed}]
        time.sleep(4.0)
        if(received_ir_data <= 0.1):
            left_and_go()
            return
        else:
            finish()
            return
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
    global encoder_a_plus_b, motor_not_move, motor_not_move_time
    motor_not_move_time = 0
    encoder_p = 0
    while not rospy.is_shutdown():
        if subscribe_data_queue:
            # if motor_not_move_time >= not_move_threshold:
                # if s_state == "start":
                    # back(0.1)
                # motor_not_move_time = 0
            # encoder_a_data = subscribe_data_queue[-1][1]  # Get the latest encoder A data
            # encoder_b_data = subscribe_data_queue[-1][2]  # Get the latest encoder B data
            encoder_a_plus_b = subscribe_data_queue[-1][1] + subscribe_data_queue[-1][2]
            if encoder_p == encoder_a_plus_b:
                motor_not_move_time += 1
                # print("mnt:", motor_not_move_time)
            # print("Encoder A value:", encoder_a_data)
            # print("Encoder B value:", encoder_b_data)
            # print("Motor value = ", encoder_a_plus_b)
        encoder_p = encoder_a_plus_b
        time.sleep(0.01)

def ir_sensor_output():
    global received_ir_data, find_ir, find_r_ir, not_find_ir, not_find_ir_counter
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
            # print("ir: ",received_ir_data)
            if (received_ir_data >= beacon_lower_bound[beacon_select]) and (received_ir_data <= beacon_upper_bound[beacon_select]):
                not_find_ir_counter = 0
                find_ir = True
            else:
                not_find_ir_counter += 1
                # print("nfic", not_find_ir_counter)
                if not_find_ir_counter >= not_find_ir_threshold:
                    not_find_ir = True
            ir_count0 = ir_count1 = 0.0

        time.sleep(0.0001)

# Main function
def main():
    global state, light_data, received_ir_data
    rospy.init_node('cp2', anonymous=True)
    
    # Start the publisher function as a thread
    threading.Thread(target=publisher_func).start()
    threading.Thread(target=read_gpio).start()
    threading.Thread(target=encoder_data_output_loop).start()
    threading.Thread(target=ir_sensor_output).start()
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
            print("5.IR sensor value")
            print("6.Start from A")
            print("7.Start from B")
            print("8.Start from C")
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
            elif count == 5:
                print("")
                # threading.Thread(target=ir_sensor_output).start()
            elif count == 6:
                # print("start from A")
                start_from_A()
            elif count == 7:
                # print("start from B")
                start_from_B()
            elif count == 8:
                # print("start from C")
                start_from_C()
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
