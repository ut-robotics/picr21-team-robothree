

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

DEBUG=1

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
def get_thresholding():
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

                lH_rect = int(data_list[6])
                lS_rect = int(data_list[7])
                lV_rect = int(data_list[8])
                hH_rect = int(data_list[9])
                hS_rect = int(data_list[10])
                hV_rect = int(data_list[11])

        else:
            lH = 0
            lS = 195
            lV = 120
            hH = 20
            hS = 255
            hV = 255

            lH_rect = 0
            lS_rect = 195
            lV_rect = 120
            hH_rect = 20
            hS_rect = 255
            hV_rect = 255
    except:
        lH = 0
        lS = 195
        lV = 120
        hH = 20
        hS = 255
        hV = 255

        lH_rect = 0
        lS_rect = 195
        lV_rect = 120
        hH_rect = 20
        hS_rect = 255
        hV_rect = 255
    return lH,lS,lV,hH,hS,hV,lH_rect,lS_rect,lV_rect,hH_rect,hS_rect,hV_rect

lH, lS, lV, hH, hS, hV, lH_rect, lS_rect, lV_rect, hH_rect, hS_rect, hV_rect = get_thresholding()


def update_value_lh(new_value):
    # Make sure to write the new value into the global variable
    global lH
    lH = new_value


def update_value_ls(new_value):
    # Make sure to write the new value into the global variable
    global lS
    lS = new_value


def update_value_lv(new_value):
    # Make sure to write the new value into the global variable
    global lV
    lV = new_value


def update_value_hh(new_value):
    # Make sure to write the new value into the global variable
    global hH
    hH = new_value


def update_value_hs(new_value):
    # Make sure to write the new value into the global variable
    global hS
    hS = new_value


def update_value_hv(new_value):
    # Make sure to write the new value into the global variable
    global hV
    hV = new_value


def update_value_lh_rect(new_value):
    # Make sure to write the new value into the global variable
    global lH_rect
    lH_rect = new_value


def update_value_ls_rect(new_value):
    # Make sure to write the new value into the global variable
    global lS_rect
    lS_rect = new_value


def update_value_lv_rect(new_value):
    # Make sure to write the new value into the global variable
    global lV_rect
    lV_rect = new_value


def update_value_hh_rect(new_value):
    # Make sure to write the new value into the global variable
    global hH_rect
    hH_rect = new_value


def update_value_hs_rect(new_value):
    # Make sure to write the new value into the global variable
    global hS_rect
    hS_rect = new_value


def update_value_hv_rect(new_value):
    # Make sure to write the new value into the global variable
    global hV_rect
    hV_rect = new_value


def get_circle_blob():
    # blob parametres
    blobparams = cv2.SimpleBlobDetector_Params()

    blobparams.filterByArea = True
    blobparams.filterByColor = True
    blobparams.filterByInertia = False
    blobparams.filterByConvexity = False
    blobparams.filterByCircularity = False
    blobparams.minDistBetweenBlobs = 10
    blobparams.minArea = 600
    blobparams.maxArea = 1000000
    blobparams.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(blobparams)
    return detector


detector = get_circle_blob()


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
        if(DEBUG):
            make_taskbars()

        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            hsv = cv2.medianBlur(hsv, 5)
            ball_hsv, keypoints_ball = ball_detection(hsv)

            rect_hsv, keypoints_rect = rect_detection(hsv)


            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(ball_hsv, dsize=(
                    depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack(
                    (resized_color_image, depth_colormap, ball_hsv))
            else:
                images = np.hstack((ball_hsv, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('also', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('also_rect', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.imshow('also', ball_hsv)
            cv2.imshow('also_rect', rect_hsv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except:
        pass
    finally:

        # Stop streaming
        pipeline.stop()

        save_thresholding()

        print("data saved")
        print("program closed")

def rect_detection(hsv):
    lowerLimits_rect = np.array([lH_rect, lS_rect, lV_rect])
    upperLimits_rect = np.array([hH_rect, hS_rect, hV_rect])
    thresholded_rect = cv2.inRange(
                hsv, lowerLimits_rect, upperLimits_rect)
    outimage_rect = cv2.bitwise_and(hsv, hsv, mask=thresholded_rect)
    keypoints_rect = detector.detect(thresholded_rect)
            #thresholded = cv2.drawKeypoints(thresholded, keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    rectNew = cv2.drawKeypoints(outimage_rect, keypoints_rect, np.array(
                []), (255, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
    return rectNew, keypoints_rect

def ball_detection(hsv):
    lowerLimits_ball = np.array([lH, lS, lV])
    upperLimits_ball = np.array([hH, hS, hV])
    thresholded_ball = cv2.inRange(
                hsv, lowerLimits_ball, upperLimits_ball)
    thresholded_ball = cv2.morphologyEx(
                thresholded_ball, cv2.MORPH_OPEN, kernel)
    thresholded_ball = cv2.dilate(
                thresholded_ball, (2, 2), iterations=3)
    outimage_ball = cv2.bitwise_and(hsv, hsv, mask=thresholded_ball)

    keypoints_ball = detector.detect(thresholded_ball)
    ballNew = cv2.drawKeypoints(outimage_ball, keypoints_ball, np.array(
                []), (255, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    if len(keypoints_ball) > 0:
        for keypoint in keypoints_ball:
            obj_x = keypoint.pt[0]
            obj_y = keypoint.pt[1]
            coords = "x: " + str(round(obj_x, 2)) + \
                        " y: " + str(round(obj_y, 2))
            cv2.putText(ballNew, str(coords), (int(
                        obj_x-20), int(obj_y+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 200), 2)
                
    return ballNew, keypoints_ball

def save_thresholding():
    with open("trackbar_defaults_green.txt", "w") as andmed:
        andmed.write(str(lH) + "\n")
        andmed.write(str(lS) + "\n")
        andmed.write(str(lV) + "\n")
        andmed.write(str(hH) + "\n")
        andmed.write(str(hS) + "\n")
        andmed.write(str(hV) + "\n")

        andmed.write(str(lH_rect) + "\n")
        andmed.write(str(lS_rect) + "\n")
        andmed.write(str(lV_rect) + "\n")
        andmed.write(str(hH_rect) + "\n")
        andmed.write(str(hS_rect) + "\n")
        andmed.write(str(hV_rect) + "\n")


def make_taskbars():
    cv2.namedWindow("Trackbars")
    cv2.namedWindow("Trackbars_rect")
    # Trackbars for modifing low and high values to find the right colored blob
    cv2.createTrackbar("Trackbar for lH", "Trackbars",
                       lH, 255, update_value_lH)
    cv2.createTrackbar("Trackbar for lS", "Trackbars",
                       lS, 255, update_value_ls)
    cv2.createTrackbar("Trackbar for lV", "Trackbars",
                       lV, 255, update_value_lv)

    cv2.createTrackbar("Trackbar for hH", "Trackbars",
                       hH, 255, update_value_hh)
    cv2.createTrackbar("Trackbar for hS", "Trackbars",
                       hS, 255, update_value_hs)
    cv2.createTrackbar("Trackbar for hV", "Trackbars",
                       hV, 255, update_value_hv)

    cv2.createTrackbar("Trackbar for lH_rect", "Trackbars_rect",
                       lH_rect, 255, update_value_lh_rect)
    cv2.createTrackbar("Trackbar for lS_rect", "Trackbars_rect",
                       lS_rect, 255, update_value_ls_rect)
    cv2.createTrackbar("Trackbar for lV_rect", "Trackbars_rect",
                       lV_rect, 255, update_value_lv_rect)

    cv2.createTrackbar("Trackbar for hH_rect", "Trackbars_rect",
                       hH_rect, 255, update_value_hh_rect)
    cv2.createTrackbar("Trackbar for hS_rect", "Trackbars_rect",
                       hS_rect, 255, update_value_hs_rect)
    cv2.createTrackbar("Trackbar for hV_rect", "Trackbars_rect",
                       hV_rect, 255, update_value_hv_rect)


if __name__ == "__main__":
    camera_thread()
