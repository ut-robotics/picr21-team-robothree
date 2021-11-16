




import pyrealsense2 as rs
import numpy as np
import cv2
import os.path
import serial
from threading import Thread
kernel = np.ones((3, 3), np.uint8)
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
try:
    if os.path.isfile("trackbar_defaults_green.txt"):
        with open("trackbar_defaults_green.txt", "r") as data:
            data_list = data.readlines()
            lH = int(data_list[0])
            lS = int(data_list[1])
            lV = int(data_list[2])
            hH = int(data_list[3])
            hS = int(data_list[4])
            hV = int(data_list[5])


            lH_Rect = int(data_list[6])
            lS_Rect = int(data_list[7])
            lV_Rect = int(data_list[8])
            hH_Rect = int(data_list[9])
            hS_Rect = int(data_list[10])
            hV_Rect = int(data_list[11])


    else:
        lH = 0
        lS = 195
        lV = 120
        hH = 20
        hS = 255
        hV = 255


        lH_Rect = 0
        lS_Rect = 195
        lV_Rect = 120
        hH_Rect = 20
        hS_Rect = 255
        hV_Rect = 255
except:
    lH = 0
    lS = 195
    lV = 120
    hH = 20
    hS = 255
    hV = 255


    lH_Rect = 0
    lS_Rect = 195
    lV_Rect = 120
    hH_Rect = 20
    hS_Rect = 255
    hV_Rect = 255

def updateValuelH(new_value):
    # Make sure to write the new value into the global variable
    global lH
    lH = new_value

def updateValuelS(new_value):
    # Make sure to write the new value into the global variable
    global lS
    lS = new_value

def updateValuelV(new_value):
    # Make sure to write the new value into the global variable
    global lV
    lV = new_value

def updateValuehH(new_value):
    # Make sure to write the new value into the global variable
    global hH
    hH = new_value

def updateValuehS(new_value):
    # Make sure to write the new value into the global variable
    global hS
    hS = new_value

def updateValuehV(new_value):
    # Make sure to write the new value into the global variable
    global hV
    hV = new_value

def updateValuelH_Rect(new_value):
    # Make sure to write the new value into the global variable
    global lH_Rect
    lH_Rect = new_value

def updateValuelS_Rect(new_value):
    # Make sure to write the new value into the global variable
    global lS_Rect
    lS_Rect = new_value

def updateValuelV_Rect(new_value):
    # Make sure to write the new value into the global variable
    global lV_Rect
    lV_Rect = new_value

def updateValuehH_Rect(new_value):
    # Make sure to write the new value into the global variable
    global hH_Rect
    hH_Rect = new_value

def updateValuehS_Rect(new_value):
    # Make sure to write the new value into the global variable
    global hS_Rect
    hS_Rect = new_value

def updateValuehV_Rect(new_value):
    # Make sure to write the new value into the global variable
    global hV_Rect
    hV_Rect = new_value

#blob parametres
blobparams = cv2.SimpleBlobDetector_Params()

blobparams.filterByArea = True
blobparams.filterByColor = True
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 10
blobparams.minArea= 600
blobparams.maxArea= 1000000
blobparams.blobColor = 255 #või img = vc2

detector = cv2.SimpleBlobDetector_create(blobparams)

def camera_thread():
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("[INFO] Dude i cant find the camera")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)


    # Start streaming
    pipeline.start(config)


    try:

        cv2.namedWindow("Trackbars")
        cv2.namedWindow("Trackbars_Rect")
        #Trackbars for modifing low and high values to find the right colored blob
        cv2.createTrackbar("Trackbar for lH", "Trackbars", lH, 255, updateValuelH)
        cv2.createTrackbar("Trackbar for lS", "Trackbars", lS, 255, updateValuelS)
        cv2.createTrackbar("Trackbar for lV", "Trackbars", lV, 255, updateValuelV)

        cv2.createTrackbar("Trackbar for hH", "Trackbars", hH, 255, updateValuehH)
        cv2.createTrackbar("Trackbar for hS", "Trackbars", hS, 255, updateValuehS)
        cv2.createTrackbar("Trackbar for hV", "Trackbars", hV, 255, updateValuehV)

        cv2.createTrackbar("Trackbar for lH_Rect", "Trackbars_Rect", lH_Rect, 255, updateValuelH_Rect)
        cv2.createTrackbar("Trackbar for lS_Rect", "Trackbars_Rect", lS_Rect, 255, updateValuelS_Rect)
        cv2.createTrackbar("Trackbar for lV_Rect", "Trackbars_Rect", lV_Rect, 255, updateValuelV_Rect)

        cv2.createTrackbar("Trackbar for hH_Rect", "Trackbars_Rect", hH_Rect, 255, updateValuehH_Rect)
        cv2.createTrackbar("Trackbar for hS_Rect", "Trackbars_Rect", hS_Rect, 255, updateValuehS_Rect)
        cv2.createTrackbar("Trackbar for hV_Rect", "Trackbars_Rect", hV_Rect, 255, updateValuehV_Rect)


        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            lowerLimits_Ball = np.array([lH, lS, lV])
            upperLimits_Ball = np.array([hH, hS, hV])
            lowerLimits_Rect = np.array([lH_Rect, lS_Rect, lV_Rect])
            upperLimits_Rect = np.array([hH_Rect, hS_Rect, hV_Rect])
            hsv = cv2.medianBlur(hsv, 5)
            thresholded_ball = cv2.inRange(hsv, lowerLimits_Ball, upperLimits_Ball)
            thresholded_Rect = cv2.inRange(hsv, lowerLimits_Rect, upperLimits_Rect)
            thresholded_ball = cv2.morphologyEx(thresholded_ball, cv2.MORPH_OPEN,kernel)
            thresholded_ball = cv2.dilate(thresholded_ball, (2,2), iterations=3)
            outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded_ball)
            outimage_Rect = cv2.bitwise_and(hsv, hsv, mask = thresholded_Rect)
            keypoints = detector.detect(thresholded_ball)
            keypoints_Rect = detector.detect(thresholded_Rect)
            #thresholded = cv2.drawKeypoints(thresholded, keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            imgNew = cv2.drawKeypoints(color_image, keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            RectNew = cv2.drawKeypoints(outimage_Rect, keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            if len(keypoints) > 0:
                for keypoint in keypoints:
                    obj_x = keypoint.pt[0]
                    obj_y = keypoint.pt[1]
                    coords = "x: " + str(round(obj_x, 2)) + " y: " + str(round(obj_y, 2))
                    cv2.putText(imgNew, str(coords), (int(obj_x-20), int(obj_y+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 200), 2)


            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(imgNew, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap,outimage))
            else:
                images = np.hstack((imgNew, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('also', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('also_Rect', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.imshow('also', outimage)
            cv2.imshow('also_Rect', RectNew)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except:
        pass
    finally:

        # Stop streaming
        pipeline.stop()

        with open("trackbar_defaults_green.txt", "w") as andmed:
            andmed.write(str(lH) + "\n")
            andmed.write(str(lS) + "\n")
            andmed.write(str(lV) + "\n")
            andmed.write(str(hH) + "\n")
            andmed.write(str(hS) + "\n")
            andmed.write(str(hV) + "\n")

            andmed.write(str(lH_Rect) + "\n")
            andmed.write(str(lS_Rect) + "\n")
            andmed.write(str(lV_Rect) + "\n")
            andmed.write(str(hH_Rect) + "\n")
            andmed.write(str(hS_Rect) + "\n")
            andmed.write(str(hV_Rect) + "\n")

        print("data saved")
        print("program closed")


if __name__ == "__main__":
    camera_thread()