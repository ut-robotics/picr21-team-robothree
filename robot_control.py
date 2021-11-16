import struct
import time
import serial
real_robot_speeds=(0,0,0)
send_motor_speeds=(0,0,0)
thrower_speed=0
WHEEL_ANGLE=120

feedback_delimiter=0
format_send='<hhhHBH'
format_receive='<hhhH'
receive_size=struct.calcsize(format_receive)
send_size=struct.calcsize(format_send)

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
