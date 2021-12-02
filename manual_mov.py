import socket
import serial
import struct
import robot_control
import cv2
import time
max_speed=4
def manual_mov():
    cv2.namedWindow("Controller")
    while True:
        k = cv2.waitKey(1) & 0xFF

        if k == ord("w"):
            print("Forward")
            robot_control.send_motor_speeds=(0,-max_speed,max_speed)
        elif k == ord("s"):
            print("Backward")
            robot_control.send_motor_speeds=(-max_speed,max_speed,0)
        elif k == ord("d"):
            print("Right")
            robot_control.send_motor_speeds=(-max_speed,-max_speed,-max_speed)
        elif k == ord("a"):
            print("Left")
            robot_control.send_motor_speeds=(max_speed,max_speed,max_speed)
        elif k == ord("t"):
            print("Throw")
            robot_control.thrower_speed=700
        elif k == ord("q"):
            print("Break")
            break
        elif k == ord("j"):
            print("1wheel")
            robot_control.send_motor_speeds=(max_speed,0,0)
        elif k == ord("k"):
            print("2wheel")
            robot_control.send_motor_speeds=(0,max_speed,0)
        elif k == ord("l"):
            print("3wheel")
            robot_control.send_motor_speeds=(0,0,max_speed)
        elif k == ord("c"):
            robot_control.send_motor_speeds=(0,0,0)
            robot_control.thrower_speed=0
            print("stop")
    time.sleep(0.2)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    manual_mov()
