import struct
import time
import serial
import math
from camera_test import CAM_HEIGHT, CAM_WIDTH
real_robot_speeds=(0,0,0)
send_motor_speeds=(0,0,0)
thrower_speed=0
WHEEL_ANGLE=120

feedback_delimiter=0
format_send='<hhhHBH'
format_receive='<hhhH'
receive_size=struct.calcsize(format_receive)
send_size=struct.calcsize(format_send)
def wheel_linear(robot_speed, wheel_angle, middle_px=None, X=None, Y=None):
    try:
        if Y is not None and Y != 0:
            robot_direction_angle = math.degrees(math.atan((middle_px - X) / Y))
            print("robotANgle", robot_direction_angle)
            wheel_linear = robot_speed * math.cos(math.radians(robot_direction_angle - wheel_angle))
        else:
            wheel_linear = robot_speed * math.cos(math.radians(90 - wheel_angle))
        return wheel_linear
    except:
        if wheel_angle==0:
            return robot_speed
        else:
            return 0
    # sd:right:middle:left
def set_omnimovement(speed, X=None, Y=None):
    send_motor_speeds[0] = wheel_linear(speed, WHEEL_ANGLE,     CAM_WIDTH/2, X, Y) #parem
    send_motor_speeds[1] = wheel_linear(speed, 0,               CAM_WIDTH/2, X, Y) #keskmine
    send_motor_speeds[2] = wheel_linear(speed, 360-WHEEL_ANGLE, CAM_WIDTH/2, X, Y) #vasak
def sending():
    global send_motor_speeds
    while True:
        try:
            mainboard= serial.Serial(port="COM7",
                baudrate=115200)
            print("done")
            break
        except:
            print("miskit juhtus")
    while True:
        time.sleep(0.1)
        while mainboard.inWaiting() > 0:
                receive_struct = mainboard.read(receive_size)
                receive_array=struct.unpack(format_receive,receive_struct)
                real_robot_speeds[0]=receive_array[0]
                real_robot_speeds[1]=receive_array[1]
                real_robot_speeds[2]=receive_array[2]
                feedback_delimiter=receive_array[3]
        send_struct=struct.pack(format_send, send_motor_speeds[0],send_motor_speeds[1],thrower_speed,0,0xAAAA)
        mainboard.write(send_struct)
