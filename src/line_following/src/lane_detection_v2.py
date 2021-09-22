#!/usr/bin/env python
import cv2
import numpy as np
from timeit import default_timer as timer
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy

import sys
print(sys.version)
print(cv2.__version__)

height = 480
width = 640
global_frame = np.zeros((height,width,3), np.uint8)
def calculate_lane_pose(frame):
    ROI_TOP_HEIGHT_PERCENTAGE = 0.5
    ROI_TOP_WIDTH_PERCENTAGE = 0.2
    ROI_BOTTOM_WIDTH_PERCENTAGE = 0.3
    ROI_BOTTOM_HEIGHT_PERCENTAGE = 0.1

    MIN_OBJECT_CONTOUR_AREA = 250
    MIN_OBJECT_LENGTH = 100

    com_x = 0
    com_y = 0

    contour_number = 0

    # Start the timer to count time-lapse
    start = timer()

    # Uncomment the following line to get input from image
    # frame = img
    ret = True
    if ret == True:

        # The following conditions would reset the contour
        # state everytime a new frame is loaded
        #
        # Initial value of the averaged center
        # point of the white lane contour
        com_x = 0
        com_y = 0
        # At the beginning, the number of the
        # contours of the detected white lane is zero
        contour_number = 0

        # Resize the input image
        scale_percent = 50
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        # Get gray scale image
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

        # Apply gaussian blur on the image to reduce noise
        blur_gray = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)

        #
        # Get derivative image
        #
        # set the kernel size of the derivative filter
        ksize = 3
        # get image derivatives along x direction
        gX = cv2.Sobel(blur_gray, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=ksize)
        # get image derivatives along y direction
        gY = cv2.Sobel(blur_gray, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=ksize)
        # the gradient magnitude images are now of the floating point data
        # type, so we need to take care to convert them back a to unsigned
        # 8-bit integer representation so other OpenCV functions can operate
        # on them and visualize them
        gX = cv2.convertScaleAbs(gX)
        gY = cv2.convertScaleAbs(gY)
        # combine the gradient representations into a single image
        combined = cv2.addWeighted(gX, 1.0, gY, 1.0, 0)

        # Get area of interest
        gray_roi = combined
        triangle = np.array([
            [(int(width*ROI_BOTTOM_WIDTH_PERCENTAGE), int(height*(1-ROI_BOTTOM_HEIGHT_PERCENTAGE))),
             (int(width*(1-ROI_BOTTOM_WIDTH_PERCENTAGE)), int(height*(1-ROI_BOTTOM_HEIGHT_PERCENTAGE))),
             (int(width*(1-ROI_TOP_WIDTH_PERCENTAGE)), int(height*ROI_TOP_HEIGHT_PERCENTAGE)),
             (int(width*ROI_TOP_WIDTH_PERCENTAGE), int(height*ROI_TOP_HEIGHT_PERCENTAGE))
            ]
        ])
        mask_roi = np.zeros_like(gray_roi)
        cv2.fillPoly(mask_roi, triangle, 255)
        area_of_interest = cv2.bitwise_and(gray_roi, mask_roi)
        # Apply thresholding
        (thresh, img_thresh) = cv2.threshold(area_of_interest, 80, 255, cv2.THRESH_BINARY)

        # apply erode
        kernel = np.ones((3,3), np.uint8)
        img_erosion = cv2.erode(img_thresh, kernel, iterations=1)
        kernel = np.ones((5,5), np.uint8)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

        # Find the contours
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

         # Find the convex hull from contours and draw it on the original image.
        convex_hull = img_thresh.copy()
        for i in range(len(contours)):
            # get convex hull of each contour
            hull = cv2.convexHull(contours[i])
            # get the aspect ratio (x,y,w,h) of the contour
            aspect_ratio = cv2.boundingRect(hull)
            # get the width of the contour
            w = aspect_ratio[2]
            # get the height of the contour
            h = aspect_ratio[3]
            # Calculate contour length
            object_length = np.sqrt((w*w) + (h*h))
            # Only contour with a certain length is accepted.
            # In this case, the lane should be significantly longer than noises
            if object_length < MIN_OBJECT_LENGTH:
                # contours that are too short are black colored
                cv2.drawContours(convex_hull, [hull], -1, (0, 0, 0), -1)
            else:
                # Draw the detected lane contours
                cv2.drawContours(convex_hull, [hull], -1, (255, 0, 0), -1)
                # Get the center point of the contour
                M = cv2.moments(hull)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # Add the detected contour center point value to the
                # averaged center value variable
                com_x = com_x + cx
                com_y = com_y + cy
                # Count the number of contour
                contour_number = contour_number + 1
        # Create exception if no contour is detected.
        # If contours detected, use the contour_number contour to apply
        # simple averaging of the whole contours center point
        try:
            com_x = com_x / contour_number
            com_y = com_y / contour_number
        except ZeroDivisionError:
            com_x = 0
            com_y = 0
        # If no contour detected, this should be the default condition,
        # which affects the moving direction of the robot.
        # In this case, the robot should not turn
        if contour_number == 0:
            com_x = int(convex_hull.shape[1]/2)
            com_y = 0
        # Draw the center point of the average contour midpoint
        cv2.circle(convex_hull, (int(com_x),int(com_y)), 3, (255, 255, 255), 5)
        cv2.circle(convex_hull, (int(com_x),int(com_y)), 2, (0, 0, 0), 2)

        # Display the resulting frame
        cv2.imshow('Frame', frame)
        cv2.imshow('thresh', img_thresh)
        cv2.imshow('erode', img_erosion)
        cv2.imshow('dilate', img_dilation)
        cv2.imshow('convex_hull', convex_hull)

        end = timer()
        # Time in seconds, e.g. 5.38091952400282
        print("computation speed : ", (end - start), "seconds")

        angle  = 0.0
        angle = np.arctan2(
                    # get the origin of the coordinate
                    # as the origin of the camera view
                    convex_hull.shape[0] - com_y,
                    com_x - int(convex_hull.shape[1]/2)
                ) * 180 / np.pi
        angle = 90 - angle
        return com_x, com_y, angle

def camera_callback(self,data):

    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)

    height, width, channels = global_frame.shape
    print(height)
    cv2.imshow("Original", global_frame)

def lane_pose_publisher():
    # cap = cv2.VideoCapture('Video with sun and shade.mp4')
    # cap = cv2.VideoCapture(0)
    # Check if camera opened successfully
    # if (cap.isOpened() == False):
    #    print("Error opening video stream or file")

    # set which topic to publish the data
    x_pos_pub = rospy.Publisher('lane_position_x', Float64, queue_size=10)
    y_pos_pub = rospy.Publisher('lane_position_y', Float64, queue_size=10)
    angle_pub = rospy.Publisher('lane_angle_twist', Float64, queue_size=10)

    # Set the node name
    rospy.init_node('lane_pose_publisher', anonymous=True)

    # set subscriber for images
    rospy.Subscriber(
            '/camera/image_raw/compressed',
            CompressedImage,
            camera_callback,
            queue_size = 1)

    # set rate
    rate = rospy.Rate(1000) # 1000hz

    while (1):
        # read captured frame
        # ret, frame = cap.read()

        # Update the pose calculation
        try:
            pos_x, pos_y, angle = calculate_lane_pose(global_frame)
        except TypeError:
            pos_x = 0
            pos_y = 0
            angle = 0
            break

        rospy.loginfo(pos_x)
        rospy.loginfo(pos_y)
        rospy.loginfo(angle)

        # publish the position and orientation calculation
        x_pos_pub.publish(pos_x)
        y_pos_pub.publish(pos_y)
        angle_pub.publish(angle)
        rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        lane_pose_publisher()
    except rospy.ROSInterruptException:
        pass
