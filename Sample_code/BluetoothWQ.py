# Change: Added video feed

from imp import is_frozen
import socket
import time
import keyboard
import stretch_body.robot
import serial
import math
import atexit

import numpy as np
import pickle as pkl
import keyboard
import os

import pyrealsense2 as rs
import numpy as np
import cv2

###############################################
##      Open CV and Numpy integration        ##
###############################################
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
###############################################
##      Open CV and Numpy integration        ##
###############################################

#os.nice(19)

def kill_robot():
    #robot.stop()
    #time.sleep(2)
    print()
    print('Emergency Stopped')
    print()
    robot.base.set_rotational_velocity(0)
    robot.base.set_translate_velocity(0)
    robot.arm.set_velocity(0)
    robot.lift.set_velocity(0)
    robot.end_of_arm.move_by('stretch_gripper',0)
    robot.end_of_arm.move_by('wrist_yaw',0)
    robot.end_of_arm.move_by('wrist_pitch',0)
    robot.push_command()
atexit.register(kill_robot)

#ser=serial.Serial('/dev/ttyUSB3',baudrate=115200,timeout=1)
#ser=serial.Serial('/dev/rfcomm0',baudrate=115200,timeout=1)
#ser=serial.Serial('/dev/tty.ESP32test-ESP32SPP',baudrate=115200,timeout=0.05)


#participant setup steps
import random
participant_num =  1 
random.seed(participant_num) 
tasks = {1: "bottle", 2: "light", 3: "clean leg", 4: "blanket", 5: "web interface comparison"}
num_tasks = 5
task_start_num = int(input("Enter Starting Task Number: "))
trial_num = int(input("Enter Trial Num: "))
random_order_list = random.sample(range(1,1+num_tasks), num_tasks)
print("Do Task:", tasks[random_order_list[task_start_num]])
datafilepath = str(participant_num) + '_' + str(random_order_list[task_start_num]) + '_' + str(trial_num)

# global variables
robot=stretch_body.robot.Robot()
robot.startup()


'''
joints = robot.end_of_arm.joints
print(joints)

#robot.end_of_arm.motors["wrist_pitch"].motor.pretty_print()
robot.end_of_arm.motors["wrist_yaw"].motor.enable_pwm()
print()
robot.end_of_arm.motors["wrist_yaw"].motor.set_pwm(100)
time.sleep(2)
robot.end_of_arm.motors["wrist_yaw"].motor.set_pwm(0)

robot.end_of_arm.motors["wrist_pitch"].motor.enable_vel()
robot.end_of_arm.motors["wrist_pitch"].motor.set_vel(50)
print()
robot.end_of_arm.motors["wrist_pitch"].motor.pretty_print()

print("Enabled Velocity control")
time.sleep(1)
robot.end_of_arm.motors["wrist_pitch"].motor.set_vel(0.0)
#time.sleep(5)
#robot.end_of_arm.motors["stretch_gripper"].follow_trajectory(v_r = 0.0)
'''
#robot.home() #don't use this - just run "python home.py"
#robot.stow()
robot.lift.set_soft_motion_limit_min(0.2,limit_type='user')
robot.lift.set_soft_motion_limit_max(0.98,limit_type='user')






def interpolation(x, y1, y2, x1, x2): #x1  = LOW, #x2 = HIGH, #x is the current angle of head, all must be positive
    y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)
    if y > y2: 
        y = y2
    elif y1 > y:
        y = y1 
    print(x, y, x1, x2, y1, y2)
    return y 

# speed scaling factors
base_min_translation_speed = 0 #m/s
base_max_translation_speed = 0.5 #m/s
base_min_rotation_speed = 0 #rad/s
base_max_rotation_speed = 0.8 #rad/s
arm_min_lift_speed = 0 #m/s
arm_max_lift_speed = 0.15 #m/s
arm_min_extend_speed = 0 #m/s
arm_max_extend_speed = 0.15 #m/s

wrist_factor=4
gripper_factor=3

v_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['vel'] #3.0
a_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['accel'] #8.0


state=0
num_state=4
head_control=False

zero_X=0
zero_Y=0
zero_Z=0

threshold_start= 15
angle_range = 20


host = '172.26.163.219' #Server ip 1075
# host = '172.26.166.129' #Server ip 1082
port = 4000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.0)

def connect_socket():
    s.bind((host, port))
    print("Server Started")
    # Start streaming
    pipeline.start(config)

def disconnect_socket():    
    c.close()
    robot.stop()


# state 0: base movement
# state 1: arm movement
# state 2: wrist
# state 3: gripper movement
# (state 4: stop)

def vid_feed():
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    color_colormap_dim = color_image.shape
    color_image = cv2.rotate(color_image, cv2.cv2.ROTATE_90_CLOCKWISE)
    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)

def update_mode(msg):
    global state, head_control, zero_X, zero_Y, zero_Z
    global roll_threshold_L, roll_threshold_H, pitch_threshold_L, pitch_threshold_H

    if (msg == "drive"):
        state = 0 
    elif (msg == "arm"):
        state = 1
    elif (msg == "wrist"):
        state = 2
    elif (msg == "gripper"):
        state = 3
    elif (msg == "stop"):  #stop control using hat 
        head_control=False
    elif (msg == "start"): #start control using hat and calibrate
        head_control=True
        robot.startup()
        n = 5
        data = np.mean(np.array(all_acc_data)[-n:,0:5], axis = 0) #average last 5 accelerometer values
        zero_X = data[0]
        zero_Y = data[1] #pitch
        zero_Z = data[2] #roll
        roll_threshold_L= -threshold_start + zero_Z
        roll_threshold_H= threshold_start + zero_Z
        pitch_threshold_L = -threshold_start + zero_Y
        pitch_threshold_H = threshold_start + zero_Y
    elif (msg == "end"): #end task
        outfile = open(datafilepath, 'wb')
        data_dict = {'data' : all_acc_data, 'mode_data' : mode_data}
        pkl.dump(data_dict, outfile, protocol=2)
        outfile.close()
        robot.stop()
        print('Saved Data')
        print('Task Number Completed:', task_start_num, "  Trial Number Completed:", trial_num)
    



#initialization 
#get first accelerometer value 
#initialize()


all_acc_data = []
mode_data = []
buffer = []
message_length = 13

send_frequency = 10 #Hz, shouldn't be more than sampling frequency of accelerometer
send_period = 1e6/send_frequency
last_send_time = time.time()
X = 0
Y = 0 
Z = 0

samp_freq = 50 #Hz
allowable_lag = 0.1 #20-percent 
allowable_time_diff = 1e6/samp_freq + (1e6/samp_freq)*allowable_lag
last_read_time = time.time()


connect_socket()

ser=serial.Serial('/dev/rfcomm0',baudrate=115200,timeout=1) #connect to hat
while(1):   
    curr_time = time.time()*1e6
    while ser.in_waiting > 0: #this checks to see if a byte is waiting to be read 
        buffer.append(ser.read())
    if len(buffer) >= message_length: #this means a full message is waiting to be processed
        # byte0 = int.from_bytes(buffer[0], byteorder='little')
        byte0 = int(''.join(reversed(buffer[0])).encode('hex'),16)
        byte1 = int(''.join(reversed(buffer[1])).encode('hex'),16)
        byte2 = int(''.join(reversed(buffer[2])).encode('hex'),16)

        if byte0 == 255 and byte1 == 255 and byte2 == 255:
            X = round(int(''.join(reversed(buffer[3]+buffer[4])).encode('hex'),16)/100,2) 
            Y = round(int(''.join(reversed(buffer[5]+buffer[6])).encode('hex'),16)/100 - 180,2) 
            Z = round(int(''.join(reversed(buffer[7]+buffer[8])).encode('hex'),16)/100 - 180,2) 
            timestep = int(''.join(reversed(buffer[9]+buffer[10]+buffer[11]+buffer[12])).encode('hex'),16)
            data = [X, Y, Z, timestep, time.time()]
            read_time_diff = curr_time - last_read_time
            if len(all_acc_data) == 0:
                last_timestep = timestep
            pico_time_diff = timestep - last_timestep
            last_timestep = timestep
            if abs(X) <= 360 and abs(Y) <= 180 and abs(Z) <= 180 and pico_time_diff < allowable_time_diff and timestep!=0:
                all_acc_data.append(data)
                #print(read_time_diff)
                last_read_time = curr_time
            else:
                print("corrupted accelerometer data", X, Y, Z, pico_time_diff, allowable_time_diff)
            for n in range(0, message_length):
                buffer.pop(0)
        else:
            buffer.pop(0)
    
    send_time_diff = curr_time - last_send_time
    if send_time_diff > send_period and np.shape(all_acc_data)[0] > 10:
        last_send_time = curr_time
        #average last n accelerometer values 
        n = 5
        data = np.mean(np.array(all_acc_data)[-n:,0:5], axis = 0)
        
        #this is where you send the data to the robot  
        try:
            msg, addr = s.recvfrom(1024)
            msg = msg.decode('utf-8')
            print("Message from: " + str(addr))
            print("From connected user: " + msg)
            print("Sending: " + msg)
            s.sendto(msg.encode('utf-8'), addr)
            update_mode(msg)
        except socket.error:
            pass
        #print("x: ",data[0]," y: ",data[1]," z: ",data[2], "mode: ", state)
        yaw=data[0]
        pitch=data[1]
        roll=data[2]
        # button=data[3]


        if (head_control==False):
            robot_command = 's'
            robot.stop()
        elif state== 0:
            if abs(roll) > abs(pitch):
                robot.base.set_translate_velocity(0)
                if roll>roll_threshold_H:
                    speed = -interpolation(roll, base_min_rotation_speed, base_max_rotation_speed, roll_threshold_H, roll_threshold_H+angle_range)
                    robot.base.set_rotational_velocity(speed)
                    robot_command = 'br'
                elif roll<roll_threshold_L:
                    speed = interpolation(roll, base_min_rotation_speed, base_max_rotation_speed, roll_threshold_L, roll_threshold_L-angle_range)
                    robot.base.set_rotational_velocity(speed)
                    robot_command = 'bl'
                else: 
                    robot_command = 'bw'
            else:
                robot.base.set_rotational_velocity(0)
                if pitch>pitch_threshold_H:
                    speed = interpolation(pitch, base_min_translation_speed, base_max_translation_speed, pitch_threshold_H, pitch_threshold_H+angle_range)
                    robot.base.set_translate_velocity(speed)
                    robot_command = 'bf'
                elif pitch<pitch_threshold_L:
                    speed = -interpolation(pitch, base_min_translation_speed, base_max_translation_speed, pitch_threshold_L, pitch_threshold_L-angle_range)
                    robot.base.set_translate_velocity(speed)
                    robot_command = 'bb'
                else: 
                    robot_command = 'bw'
        
        elif state== 1:
            #speed0=(abs(roll)-25)/100
            #speed1=(abs(pitch)-25)/100
            if abs(roll) > abs(pitch):
                robot.lift.set_velocity(0)
                if roll>roll_threshold_H:
                    speed = interpolation(roll, arm_min_extend_speed, arm_max_extend_speed, roll_threshold_H, roll_threshold_H+angle_range)
                    robot.arm.set_velocity(speed)
                    robot_command = 'ae'
                elif roll<roll_threshold_L:
                    speed = -interpolation(roll, arm_min_extend_speed, arm_max_extend_speed, roll_threshold_L, roll_threshold_L-angle_range)
                    robot.arm.set_velocity(speed)
                    robot_command = 'ar'
                else: 
                    robot_command = 'aw'
            else:
                robot.arm.set_velocity(0)
                if pitch>pitch_threshold_H:
                    speed = -interpolation(pitch, arm_min_extend_speed, arm_max_extend_speed, pitch_threshold_H, pitch_threshold_H+angle_range)
                    robot.lift.set_velocity(speed)
                    robot_command = 'ad' 
                elif pitch<pitch_threshold_L:
                    speed = interpolation(pitch, arm_min_extend_speed, arm_max_extend_speed, pitch_threshold_L, pitch_threshold_L-angle_range)
                    robot.lift.set_velocity(speed)
                    robot_command = 'au' 
                else: 
                    robot_command = 'aw'
                
                

        elif state== 2:
            wpitch=robot.end_of_arm.status['wrist_pitch']['pos']
            wyaw=robot.end_of_arm.status['wrist_yaw']['pos']
            if abs(roll) > abs(pitch):
                robot.end_of_arm.move_by('wrist_pitch', 0, 0, 0)
                #robot.end_of_arm.move_by('wrist_yaw',0,0, 0)
                if roll>roll_threshold_H:
                    speed=(roll-roll_threshold_H)/wrist_factor
                    robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
                    robot_command = 'wr'
                elif roll<roll_threshold_L:
                    speed=(roll-roll_threshold_L)/wrist_factor
                    robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
                    robot_command = 'wl'
                else: 
                    speed = 0
                    robot_command = 'ww'
            else:
                robot.end_of_arm.move_by('wrist_yaw', 0, 0, 0)
                #robot.end_of_arm.move_by('wrist_pitch', 0, 0, 0)
                if pitch>pitch_threshold_H:
                    speed=(pitch-pitch_threshold_H)/wrist_factor
                    robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)
                    robot_command = 'wd'
                elif pitch<pitch_threshold_L:
                    speed=(pitch-pitch_threshold_L)/wrist_factor
                    robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)
                    robot_command = 'wu'
                else:
                    speed = 0 
                    robot_command = 'ww'
                    
        if state== 3:
            if pitch>pitch_threshold_H:
                speed=(pitch-pitch_threshold_H)/gripper_factor
                robot.end_of_arm.move_by('stretch_gripper',speed)
                robot_command = 'go'
            elif pitch<pitch_threshold_L:
                speed=(pitch-pitch_threshold_L)/gripper_factor
                robot.end_of_arm.move_by('stretch_gripper',speed)
                robot_command = 'gc'
            else: 
                robot_command = 'gw'

        d = [time.time(), state, robot_command]
        mode_data.append(d)
        if head_control == True:
            print("pitch: ", pitch, "pthreshes: ", pitch_threshold_L, pitch_threshold_H)
            #print("yaw: ", yaw, "ythreshes: ", yaw_threshold_L, yaw_threshold_H)
            print("Robot command:", robot_command)
            print()
        robot.push_command()
    vid_feed()
        #if state== 4:
        #    robot.stop()

        # robot.arm.move_by(-0.1)
        # robot.base.rotate_by(0.3)
        # robot.push_command()
        # time.sleep(2.0)

        # robot.base.translate_by(-0.3)
        # time.sleep(2.0)


        
