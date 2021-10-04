import struct
import time
import serial
robot_control = {"motor1":0,"motor2":0,"motor3":0,"thrower":0}
real_robot = {"motor1":0,"motor2":0,"motor3":0}
speeds=(0,0,0)
WHEEL_ANGLE=120

feedback_delimiter=0
format_send='<hhhHBH'
format_receive='<hhhH'
receive_size=struct.calcsize(format_receive)
send_size=struct.calcsize(format_send)
def omni_movement(self, speed, middle_px, X=None, Y=None):
    speeds[0] = self.wheelLinearVelocity(speed, self.right_wheel_angle, middle_px, X, Y) #parem
    speeds[1] = self.wheelLinearVelocity(speed, self.middle_wheel_angle, middle_px, X, Y) #keskmine
    speeds[2] = self.wheelLinearVelocity(speed, self.left_wheel_angle, middle_px, X, Y) #vasak
def sending():
    global speeds
    while True:
        try:
            mainboard= serial.Serial(port="COM7",
                baudrate=115200)
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
        asi=struct.pack(format_send, speeds[0],speeds[1],speeds[2],robot_control["thrower"],0,0xAAAA)
        mainboard.write(asi)
        print(asi)
