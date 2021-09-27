import socket
import serial
import struct
import robot_control
import cv2
import time
Speed=10
def manual_mov():
    cv2.namedWindow("Controller")
    while True:
        k = cv2.waitKey(1) & 0xFF

        if k == ord("w"):
            print("Forward")
            robot_control.speeds=(Speed,-Speed,0)
        elif k == ord("s"):
            print("Backward")
            robot_control.speeds=(-Speed,Speed,0)
        elif k == ord("d"):
            print("Right")
            robot_control.speeds=(Speed,Speed,Speed)
        elif k == ord("a"):
            print("Left")
            robot_control.speeds=(-Speed,-Speed,-Speed)
        elif k == ord("t"):
            print("Throw")
            robot_control.robot_control["thrower"]=Speed
        elif k == ord("q"):
            print("Break")
            break
        else:
            robot_control.speeds=(0,0,0)
            robot_control.robot_control["thrower"]=0
    time.sleep(0.2)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    manual_mov()
