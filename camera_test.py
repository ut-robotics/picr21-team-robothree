'''
Skip to content

    Why GitHub?
                          


                    
Team
Enterprise
Explore
                      

                    
Marketplace
Pricing
                       


                        

Sign in
Sign up
IntelRealSense /
librealsense

5.4k

    3.8k

Code
Issues 313
Pull requests 77
Actions
Wiki
Security

    Insights

librealsense/wrappers/python/examples/opencv_viewer_example.py /
@nohayassin
nohayassin demo fixes
Latest commit 8f20392 on May 6
History
2 contributors
@nohayassin
@Nir-Az
76 lines (59 sloc) 2.55 KB'''
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################






import pyrealsense2 as rs
import numpy as np
import cv2
import os.path

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

if os.path.isfile("trackbar_defaults_green.txt"):
    with open("trackbar_defaults_green.txt", "r") as andmed:
        andmed_list = andmed.readlines()
        lH = int(andmed_list[0])
        lS = int(andmed_list[1])
        lV = int(andmed_list[2])
        hH = int(andmed_list[3])
        hS = int(andmed_list[4])
        hV = int(andmed_list[5])
else:
    lH = 0
    lS = 195
    lV = 120
    hH = 20
    hS = 255
    hV = 255

def updateValueH(new_value):
    # Make sure to write the new value into the global variable
    global lH
    lH = new_value

def updateValueS(new_value):
    # Make sure to write the new value into the global variable
    global lS
    lS = new_value

def updateValueV(new_value):
    # Make sure to write the new value into the global variable
    global lV
    lV = new_value

def updateValuehH(new_value):
    # Make sure to write the new value into the global variable
    global hH
    hH = new_value

def updateValueSh(new_value):
    # Make sure to write the new value into the global variable
    global hS
    hS = new_value

def updateValueVh(new_value):
    # Make sure to write the new value into the global variable
    global hV
    hV = new_value

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


found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:

    cv2.namedWindow("Trackbars")

    #Trackbars for modifing low and high values to find the right colored blob
    cv2.createTrackbar("Trackbar for lH", "Trackbars", lH, 180, updateValueH)
    cv2.createTrackbar("Trackbar for lS", "Trackbars", lS, 255, updateValueS)
    cv2.createTrackbar("Trackbar for lV", "Trackbars", lV, 255, updateValueV)

    cv2.createTrackbar("Trackbar for hH", "Trackbars", hH, 180, updateValuehH)
    cv2.createTrackbar("Trackbar for hS", "Trackbars", hS, 255, updateValueSh)
    cv2.createTrackbar("Trackbar for hV", "Trackbars", hV, 255, updateValueVh)

    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lowerLimits = np.array([lH, lS, lV])
        upperLimits = np.array([hH, hS, hV])

        thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)

        outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded)

        keypoints = detector.detect(thresholded)

        imgNew = cv2.drawKeypoints(color_image, keypoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("Threshold", thresholded)

        if len(keypoints) > 0:
            for keypoint in keypoints:
                obj_x = keypoint.pt[0]
                obj_y = keypoint.pt[1]
                coords = "x: " + str(round(obj_x, 2)) + " y: " + str(round(obj_y, 2))
                cv2.putText(color_image, str(coords), (int(obj_x-20), int(obj_y+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 200), 2)


        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
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
    print("data saved")
    print("program closed")








'''
    © 2021 GitHub, Inc.
    Terms
    Privacy
    Security
    Status
    Docs

    Contact GitHub
    Pricing
    API
    Training
    Blog
    About

'''
