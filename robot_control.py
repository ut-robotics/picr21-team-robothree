import struct
import time
import serial
robot_control = {"motor1":0,"motor2":0,"motor3":0,"thrower":0}
real_robot = {"motor1":0,"motor2":0,"motor3":0}
speeds=(robot_control["motor1"],robot_control["motor2"],robot_control["motor3"])
feedback_delimiter=0
format_send='<hhhHBH'
format_receive='<hhhH'
receive_size=struct.calcsize(format_receive)
send_size=struct.calcsize(format_send)
def sending():
    while True:
        try:
            mainboard= serial.Serial(port="COM3",
                baudrate=115200,
                bytesize=serial.EIGHTBITS)
            print("done")
            break
        except:
            pass
    while True:
        time.sleep(0.1)
        while mainboard.inWaiting() > 0:
                data = mainboard.read(receive_size)
                receive=struct.unpack(format_receive,data)
                real_robot["motor1"]=receive[0]
                real_robot["motor2"]=receive[1]
                real_robot["motor3"]=receive[2]
                feedback_delimiter=receive[3]
        struct.pack(format_send, robot_control["motor1"],robot_control["motor2"],robot_control["motor3"],robot_control["thrower"],0,0xAAAA)
        print(struct)
