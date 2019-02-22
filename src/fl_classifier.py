#!/usr/bin/env python

# Python classics
import sys, time

# OpenCV
import cv2
import csv
from cv_bridge import CvBridge
# ROS classics
import rospy
import roslib
import os.path

# ROS messages
from std_msgs.msg import Int64
from sensor_msgs.msg import CompressedImage, Image

# Video Editing
import numpy as np
import argparse
import sys

VERBOSE=True

class fl_classifier():
    def __init__(self):


        ###############################################
        ###### CLASSIFIER PARAMETERS ##################
        ###############################################

        ##################
        ##### TRAINING PART
        self.isClassified = False  # 0 for orange, 1 for pink

        self.morphOpSize = 5
        self.maxObjects = 1
        self.minObjectArea = 1500
        self.blur_size = 3
        self.lower = np.array([0, 80, 0], np.uint8)
        self.upper = np.array([180, 255, 255], np.uint8)
        self.name = "Object!"

        # Color or grey classifier.
        self.colorBool = False
        # KNN MODEL
        self.knn = cv2.ml.KNearest_create()

        if self.isClassified == False:
            # classify training pictures
            self.classify()
            print('Classification successful')
            self.isClassified = True

        ##################
        ##### TESTING PART
        self.state = 'Classify'
        # Number of meighbors to consider
        self.K = 3




        self.bridge = CvBridge()

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.classified_sign_str = ''
        self.classified_sign_int = 0
        self.sign_classification = rospy.Publisher("/sign_classification",Int64,queue_size=1)

        self.isSimulation = False
        if self.isSimulation:
            self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed", Image, self.callback_classifier,queue_size=1)
        else:
            self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback_classifier,queue_size=1)
        if VERBOSE:
            print("subscribed to /raspicam_node/image/compressed")

    def classify(self):
        global img_hsv
        global imgTrack
        global img_Crop
        global res
        ### Load training images and labels
        with open('./imgs/train.txt', 'rb') as train:
            reader_train = csv.reader(train)
            lines_train = list(reader_train)

        # read in training labels
        train_labels = np.array([np.int32(lines_train[i][1]) for i in range(len(lines_train))])

        train_list = []
        for i in range(0, len(lines_train)):
            img_bgr = cv2.imread("./imgs/" + lines_train[i][0] + ".png", 1)
            imgTrack = img_bgr.copy()
            img_blur = cv2.GaussianBlur(img_bgr, (self.blur_size, self.blur_size), 0)
            img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, self.lower, self.upper)
            res = cv2.bitwise_and(imgTrack, imgTrack, mask=mask)
            imgMorphOps = self.morphOps(mask, self.morphOpSize)
            centers = self.findObjects(imgMorphOps)
            img_lil = cv2.resize(img_Crop, (40, 40))
            img_gray = cv2.cvtColor(img_lil, cv2.COLOR_BGR2GRAY)

            if (self.colorBool):
                train_list.append(np.array(img_lil))
            else:
                train_list.append(np.array(img_gray))
           
        train2 = np.asarray(train_list)
        if (self.colorBool):
            train_data = train2.flatten().reshape(len(lines_train), 40 * 40 * 3)
        else:
            train_data = train2.flatten().reshape(len(lines_train), 40 * 40)
        train_data = train_data.astype(np.float32)

        ### Train classifier
        self.knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    def callback_classifier(self, ros_data):
        global img_hsv
        global imgTrack
        global img_Crop
        global res        # print('hello world')
        if self.state == 'Classify' and self.isClassified == True:
            # print('Classification of a new image in progress')
            if self.isSimulation == True:

                # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
                img_bgr = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            else:
                #### direct conversion to CV2 ####
                np_arr = np.fromstring(ros_data.data, np.uint8)
                # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
                img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            #DEBUG
            # print('there is a file= ' ,os.path.isfile("./imgs/image.png"))
            # img_bgr = cv2.imread("./imgs/image.png")


            imgTrack = img_bgr.copy()
            # print('imgTrack = ',imgTrack)
            img_blur = cv2.GaussianBlur(img_bgr, (self.blur_size, self.blur_size), 0)
            img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, self.lower, self.upper)
            res = cv2.bitwise_and(imgTrack, imgTrack, mask=mask)
            imgMorphOps = self.morphOps(mask, self.morphOpSize)
            centers = self.findObjects(imgMorphOps)
            img_lil = cv2.resize(img_Crop, (40, 40))
            img_gray = cv2.cvtColor(img_lil, cv2.COLOR_BGR2GRAY)

            # cv2.imwrite('img_lil.png', img_lil)

            if (self.colorBool):
                test_img = np.asarray(img_lil)
                test_img = test_img.flatten().reshape(1, 40 * 40 * 3)
            else:
                test_img = np.asarray(img_gray)
                test_img = test_img.flatten().reshape(1, 40 * 40)
            test_img = test_img.astype(np.float32)

            # print('self.knn type= ', type(self.knn))
            ret, results, neighbours, dist = self.knn.findNearest(test_img, self.K)
            # print('ret =', ret)
            if  0== ret:
                self.classified_sign_int =0
                self.classified_sign_str = 'Wall'
            elif 1 ==ret:
                self.classified_sign_int =1
                self.classified_sign_str = 'Left Turn'
            elif 2 ==ret:
                self.classified_sign_int = 2
                self.classified_sign_str = 'Right Turn'
            elif 3 ==ret:
                self.classified_sign_int = 3
                self.classified_sign_str = 'Dead End'
            elif 4 ==ret:
                self.classified_sign_int = 4
                self.classified_sign_str = 'STOP'
            elif 5 ==ret:
                self.classified_sign_int = 5
                self.classified_sign_str = 'Goal'
            # print('I detect a ',self.classified_sign)

            # return  0


    def morphOps(self,binaryMatrix, kernelSize):
        # Morphological operations (open and close) used to reduce noise in the acquired image.
        kernel = np.ones((kernelSize, kernelSize), np.uint8)
        tempFix = cv2.morphologyEx(binaryMatrix, cv2.MORPH_CLOSE, kernel)  # Fill in holes
        fix = tempFix  # cv2.morphologyEx(tempFix,cv2.MORPH_OPEN, kernel)             # Get rid of noise
        return fix

    def drawCOM(self,frame, x, y, name):
        cv2.circle(frame, (x, y), 5, (0, 255, 0))
        cv2.putText(frame, name, (x - 30, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    def findObjects(self,binaryMatrix):
        global img_hsv
        global imgTrack
        global img_Crop
        global res
        # Finds the location of the desired object in the image.
        output = []
        trash, contours, hierarchy = cv2.findContours(binaryMatrix, cv2.RETR_EXTERNAL,
                                                      cv2.CHAIN_APPROX_SIMPLE)  # Contours the image to find blobs of the same color
        cont = sorted(contours, key=cv2.contourArea, reverse=True)[
               :self.maxObjects]  # Sorts the blobs by size (Largest to smallest)


        # global img_hsv
        # Find the center of mass of the blob if there are any
        if hierarchy is not None:
            for i in range(0, len(cont)):
                M = cv2.moments(cont[i])
                if M['m00'] > self.minObjectArea:  # Check if the total area of the contour is large enough to care about!
                    x, y, w, h = cv2.boundingRect(cont[i])
                    rect = cv2.minAreaRect(cont[0])
                    w = int(rect[1][0])
                    x = int(M['m10'] / M['m00'])
                    y = int(M['m01'] / M['m00'])
                    # cv2.drawContours(imgTrack, cont[i], -1, (255,0,0), 3) # Draws the contour.
                    img_Crop = img_hsv[y - h / 2:y + h / 2, x - w / 2:x + w / 2].copy()
                    cv2.rectangle(imgTrack, (x - w / 2, y - h / 2), (x + w / 2, y + h / 2), (0, 255, 0), 2)
                    self.drawCOM(imgTrack, x, y, self.name)
                    if output == []:
                        output = [[x, w]]
                    else:
                        output.append[[x, w]]

        if len(output) is 0:  # We didn't find anything to track take whole image in (blank wall most likely)
            img_Crop = imgTrack.copy()

        return output


def main(args):
    '''Initializes and cleanup ros node'''
    ic = fl_classifier()
    rospy.init_node('fl_classifier', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down fl_classifier")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
