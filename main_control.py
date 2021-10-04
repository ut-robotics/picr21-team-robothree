#import camera_test
import manual_mov
import threading
import robot_control
if __name__ == "__main__":
  #  threading.Thread(target=camera_test.camera_thread)
    manual=threading.Thread(target=manual_mov.manual_mov)
    control=threading.Thread(target=robot_control.sending)
    manual.start()
    control.start()
    manual.join()


