#!/usr/bin/env python
# Detect and object with the camera.

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import sys

###################################
## VARIABLE DECLARATION AND SETUP
###################################

bridge = CvBridge()  # Bridge converts the image from ros to openCV

name = "Object!"

lower= np.array([0,80,80],np.uint8) # Array (H,S,V) for the lower threshold bound of the HSV image
upper= np.array([180,255,255],np.uint8) # Array (H,S,V) for the upper threshold bound of the HSV image
# error = np.array([13,100,100],np.uint8) # Array of error widths to create the upper and lower threshold bounds above.

modes = np.array([["wall",0],["left_bs",1],["dead_end",2],["stop",3],["goal",4],["left_bc",5],
                  ["right_bs",6],["right_bc",7],["left_gs",8],["right_gs",9]])

# Thresholds for 3 colors of signs:
lower_red1 = np.array([150, 50, 10], np.uint8)  # Array (H,S,V) for the lower threshold bound of the HSV image
upper_red1 = np.array([190, 230, 200], np.uint8)  # Array (H,S,V) for the upper threshold bound of the HSV image

lower_red2 = np.array([0, 50, 10], np.uint8)  # Array (H,S,V) for the lower threshold bound of the HSV image
upper_red2 = np.array([20, 230, 200], np.uint8)  # Array (H,S,V) for the upper threshold bound of the HSV image

lower_blue = np.array([90, 50, 10], np.uint8)  # Array (H,S,V) for the lower threshold bound of the HSV image
upper_blue = np.array([125, 230, 200], np.uint8)  # Array (H,S,V) for the upper threshold bound of the HSV image

lower_green = np.array([50, 50, 10], np.uint8)  # Array (H,S,V) for the lower threshold bound of the HSV image
upper_green = np.array([95, 230, 200], np.uint8)  # Array (H,S,V) for the upper threshold bound of the HSV image

titleTracker = "Color Tracker"  # Debugging Image Title
titleOriginal = "Original Image"  # Debugging Image Title
titleMask = "Mask Image"  # Debugging Image Title
debug = False  # True - shows the images. False - Does not show the images.

width = 360  # Width of the image, this is sent in our point message as the z-component to know the zero point in the frame.
blurSize = 9  # Blur Kernel Size
morphOpSize = 5  # Closing and Opening Kernel Size

maxObjects = 5  # Max number of object to detect.
minObjectArea = 50  # Min number of pixels for an object to be recognized.

start = False  # Set to true when first image is acquired and will start the program.

update = False  # True - When a new point has been found and can be published. False - Otherwise.

mose = False


###################################
## Function Declaration
###################################

def morphOps(binaryMatrix, kernelSize):
    # Morphological operations (open and close) used to reduce noise in the acquired image.
    kernel = np.ones((kernelSize, kernelSize), np.uint8)
    tempFix = cv2.morphologyEx(binaryMatrix, cv2.MORPH_CLOSE, kernel)  # Fill in holes
    fix = cv2.morphologyEx(tempFix, cv2.MORPH_OPEN, kernel)  # Get rid of noise
    return fix


def drawCOM(frame, x, y, name):
    cv2.circle(frame, (x, y), 5, (0, 255, 0))
    cv2.putText(frame, name, (x - 30, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


def findObjects(binaryMatrix):
    global x1
    global x2
    global y1
    global y2

    # Finds the location of the desired object in the image.
    output = []
    trash, contours, hierarchy = cv2.findContours(binaryMatrix, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)  # Contours the image to find blobs of the same color
    cont = sorted(contours, key=cv2.contourArea, reverse=True)[
           :maxObjects]  # Sorts the blobs by size (Smallest to Largest)

    # Find the center of mass of the blob if there are any
    if hierarchy is not None:
        for i in range(0, len(cont)):
            M = cv2.moments(cont[i])
            if M['m00'] > minObjectArea:  # Check if the total area of the contour is large enough to care about!
                rect = cv2.minAreaRect(cont[0])
                w = int(rect[1][0])
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                if (debug):
                    cv2.drawContours(imgTrack, cont[i], -1, (255, 0, 0), 3)  # Draws the contour.
                    drawCOM(imgTrack, x, y, name)
                    # print(rect[0][0])
                    # print(rect[0][1])
                    # print(rect)
                    x1 = int(rect[0][0]) - int(rect[1][0]/2)
                    y1 = int(rect[0][1]) + int(rect[1][1]/2)
                    x2 = int(rect[0][0]) + int(rect[1][0]/2)
                    y2 = int(rect[0][1]) - int(rect[1][1]/2)

                    cv2.rectangle(imgTrack,(x1,y1), (x2,y2),(0,255,0), 5)

                if output == []:
                    output = [[x, w]]
                else:
                    output.append([x, w])

    return output



def findObjects_crop(binaryMatrix):
    global imgCrop

    global x1
    global x2
    global y1
    global y2

    # Finds the location of the desired object in the image.
    output = []
    trash, contours, hierarchy = cv2.findContours(binaryMatrix, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)  # Contours the image to find blobs of the same color
    cont = sorted(contours, key=cv2.contourArea, reverse=True)[
           :maxObjects]  # Sorts the blobs by size (Smallest to Largest)t)

    imgCrop = imgBGR

    # Find the center of mass of the blob if there are any
    if hierarchy is not None:
        for i in range(0, len(cont)):
            M = cv2.moments(cont[i])
            if M['m00'] > minObjectArea:  # Check if the total area of the contour is large enough to care about!
                rect = cv2.minAreaRect(cont[0])
                w = int(rect[1][0])
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                if (debug):
                    cv2.drawContours(imgTrack, cont[i], -1, (255, 0, 0), 3)  # Draws the contour.
                    drawCOM(imgTrack, x, y, name)
                    # print(rect[0][0])
                    # print(rect[0][1])
                    # print(rect)
                    x1 = int(rect[0][0]) - int(rect[1][0]/2)
                    y1 = int(rect[0][1]) + int(rect[1][1]/2)
                    x2 = int(rect[0][0]) + int(rect[1][0]/2)
                    y2 = int(rect[0][1]) - int(rect[1][1]/2)

                    cv2.rectangle(imgTrack,(x1,y1), (x2,y2),(0,255,0), 5)

                if output == []:
                    output = [[x, w]]
                else:
                    output.append([x,w])


                imgCrop = imgBGR[y2:y1, x1:x2]

    return output, imgCrop


def get_image(CompressedImage):

    print("Getting image")
    # get_image is the main function to find the circles in the image. Get_image triggers each time a new image arrives.

    # All the images used to find the ball are made global so we can display them durring debugging.
    global imgBGR
    global imgHSV
    global imgBLUR
    global mask
    global imgMorphOps
    global imgTrack

    # Needed parameters from outside this function (lazy and globaling them).
    global p
    global update
    global start
    global morphOpSize
    global blurSize
    global width
    global pt

    # The "CompressedImage" is transformed to a color image in BGR space and is store in "imgBGR"
    imgBGR = bridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

    # height and width of the image to pass along to the PID controller as the reference point.
    height, width = imgBGR.shape[:2]

    # Image used to draw things on!
    imgTrack = imgBGR.copy()

    # Blur the image to reduce edges caused by noise or that are useless to us.
    imgBlur = cv2.GaussianBlur(imgBGR, (blurSize, blurSize), 0)

    # Transform BGR to HSV to avoid lighting issues.
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

    # Threshold the image using the selected lower and upper bounds of the color of the object.
    mask_red1 = cv2.inRange(imgHSV, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(imgHSV, lower_red2, upper_red2)

    mask_red = cv2.addWeighted(mask_red1, 1.0, mask_red2, 1.0, 0.0)

    mask_green = cv2.inRange(imgHSV, lower_green, upper_green)
    mask_blue = cv2.inRange(imgHSV, lower_blue, upper_blue)

    mask_bg = cv2.add(mask_green,mask_blue)

    mask = cv2.add(mask_red, mask_bg)

    # To get rid of noise and fill in gaps in our object use open and close.
    imgMorphOps = morphOps(mask, morphOpSize)

    centers = findObjects(imgMorphOps)

    # print ("centers", not centers)

    # Not always, the houghCircles function finds circle, so a None inspection is made
    if not centers:
        # print("hello")
        # If no object was found, sends bogus numbers.
        pt = Point()

        pt.x = 999
        pt.y = 999
        pt.z = 999
        update = True

    elif centers is not []:
        min_error = 1000
        for i in centers:
            # The x position of the center of the object, the width of the object, and the width of the image.
            p = Point(i[0], i[1], width)

            error = abs(p.x - p.z / 2)
            if error < min_error:
                min_error = error

                pt = Point()

                pt.x = p.x
                pt.y = p.y
                pt.z = p.z
                print(pt.x)

            # Bool to indicate the need to publish new information
            update = True


    start = True


def get_image_sim(Image):

    # get_image is the main function to find the circles in the image. Get_image triggers each time a new image arrives.

    # All the images used to find the ball are made global so we can display them durring debugging.
    global imgBGR
    global imgHSV
    global imgBLUR
    global mask
    global imgMorphOps
    global imgTrack

    # Needed parameters from outside this function (lazy and globaling them).
    global p
    global update
    global start
    global morphOpSize
    global blurSize
    global width
    global pt

    # The "Image" is transformed to a color image in BGR space and is store in "imgBGR"
    imgBGR = bridge.imgmsg_to_cv2(Image, "bgr8")
    # cv2.imshow("win", imgBGR)

    # height and width of the image to pass along to the PID controller as the reference point.
    height, width = imgBGR.shape[:2]

    # Image used to draw things on!
    imgTrack = imgBGR.copy()

    # Blur the image to reduce edges caused by noise or that are useless to us.
    imgBlur = cv2.GaussianBlur(imgBGR, (blurSize, blurSize), 0)

    # Transform BGR to HSV to avoid lighting issues.
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

    # Threshold the image using the selected lower and upper bounds of the color of the object.
    mask_red1 = cv2.inRange(imgHSV, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(imgHSV, lower_red2, upper_red2)

    mask_red = cv2.addWeighted(mask_red1, 1.0, mask_red2, 1.0, 0.0)

    mask_green = cv2.inRange(imgHSV, lower_green, upper_green)
    mask_blue = cv2.inRange(imgHSV, lower_blue, upper_blue)

    mask_bg = cv2.add(mask_green,mask_blue)

    # mask = cv2.add(mask_red, mask_bg)
    mask = cv2.inRange(imgHSV, lower, upper)

    # To get rid of noise and fill in gaps in our object use open and close.
    imgMorphOps = morphOps(mask, morphOpSize)

    centers = findObjects(imgMorphOps)
    # centers, imgCrop = findObjects_crop(imgMorphOps)

    # print ("centers", not centers)

    # Not always, the houghCircles function finds circle, so a None inspection is made
    if not centers:
        # print("hello")
        # If no object was found, sends bogus numbers.
        pt = Point()

        pt.x = 999
        pt.y = 999
        pt.z = 999
        update = True

    elif centers is not []:
        min_error = 1000
        for i in centers:
            # The x position of the center of the object, the width of the object, and the width of the image.
            p = Point(i[0], i[1], width)

            error = abs(p.x - p.z / 2)
            if error < min_error:
                min_error = error

                pt = Point()

                pt.x = p.x
                pt.y = p.y
                pt.z = p.z

            # Bool to indicate the need to publish new information
            update = True

    # Once the first image has been processed set start to True to display.
    start = True


def Init():
    # Creates the node, the publisher, and subscribes to the compressedImage.

    global pub
    pub = rospy.Publisher('image_location', Point, queue_size=10)

    # Subscriber
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, get_image)
    # rospy.Subscriber("/raspicam_node/image/compressed", Image, get_image_sim)

    # Initializate the node and gives a name, in this case, 'find_ball'
    rospy.init_node('detect_sign', anonymous=True)

    # Create a publisher that will be publishing Geometric message Points



###################################
## MAIN
###################################

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass

# Rate is used for manage the looping desired rate by using the method 'sleep'
rate = rospy.Rate(10)

# Create Debugging Windows
if (debug):
    cv2.namedWindow(titleTracker, cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(titleTracker, 620, 50)
    cv2.namedWindow(titleMask, cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(titleMask, 1240, 50)
    cv2.namedWindow(titleOriginal, cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(titleOriginal, 50, 50)
    # cv2.namedWindow("Cropped", cv2.WINDOW_AUTOSIZE)

while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    # If the first image arrived, the start = True
    if start:

        # Display the image
        if debug:
            cv2.imshow(titleOriginal, imgBGR)
            cv2.imshow(titleMask, mask)
            cv2.imshow(titleTracker, imgTrack)
            # cv2.imshow("Cropped", imgCrop)

        # If a new point was found, then update is True and the point is publish
        if update:
            print ("point",pt)
            pub.publish(pt)
            update = False

        rate.sleep()
        # k = cv2.waitKey(5)

