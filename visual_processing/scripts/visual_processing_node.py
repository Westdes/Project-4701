#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import time
import math
import rospy
import numpy as np
from armpi_pro import Misc
from armpi_pro import apriltag
from threading import RLock, Timer
from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from visual_processing.msg import Result
from visual_processing.srv import SetParam


lock = RLock()

image_sub = None
publish_en = True
size_m = (320, 240)
__isRunning = False
target_type = 'None'
target_color = 'None'
id_smallest = 'None'
color_range_list = None
pub_time = time.time()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


# fined max contour
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # walk through all
        contour_area_temp = math.fabs(cv2.contourArea(c))  # area calculation
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # filter the area larger than 10
                area_max_contour = c
    return area_max_contour, contour_area_max  # return max contour


# apriltag detection
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())


def apriltag_Detect(img, type = 0):
    global pub_time
    global publish_en
    global id_smallest

    msg = Result()
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(
        img_copy, size_m, interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for i, detection in enumerate(detections):
            tag_id = int(detection.tag_id)        # get tag_id
            corners = np.rint(detection.corners)  # get four corners

            # TODO: map the four corners from the size_m to image size
            for i in range(4):
                corners[i][0] = int(
                    Misc.map(corners[i][0], 0, size_m[0], 0, img_w))
                corners[i][1] = int(
                    Misc.map(corners[i][1], 0, size_m[1], 0, img_h))

            # TODO: get the center point (object_center_x, object_center_y, in image size)
            # Hint: use detection.center

            object_center_x = int(
                Misc.map(detection.center[0], 0, size_m[0], 0, img_w))
            object_center_y = int(
                Misc.map(detection.center[1], 0, size_m[1], 0, img_h))  # centre point

            object_angle = int(math.degrees(math.atan2(
                corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # rotation angle

            cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)
            if type == 1:
                if id_smallest == 'None' or tag_id >= id_smallest:  # reverse (321)
                    id_smallest = tag_id
                    msg.center_x = object_center_x
                    msg.center_y = object_center_y
                    msg.angle = object_angle
                    msg.data = id_smallest
            elif type == 2:
                if (id_smallest == 'None' or tag_id <= id_smallest or tag_id == 2) and id_smallest != 2:    # 2 first (213)
                    id_smallest = tag_id
                    msg.center_x = object_center_x
                    msg.center_y = object_center_y
                    msg.angle = object_angle
                    msg.data = id_smallest
            else:
                if id_smallest == 'None' or tag_id <= id_smallest:  # normal (123)
                    id_smallest = tag_id        
                    msg.center_x = object_center_x
                    msg.center_y = object_center_y
                    msg.angle = object_angle
                    msg.data = id_smallest

        id_smallest = 'None'
        publish_en = True

    if publish_en:
        if (time.time()-pub_time) >= 0.06:
            result_pub.publish(msg)  # publish
            pub_time = time.time()

        if msg.data == 0:
            publish_en = False
            result_pub.publish(msg)

    return img


# [ROI, weight]
roi = [(50, 70,  0, 160, 0.2),
       (80, 90,  0, 160, 0.3),
       (100, 115,  0, 160, 0.5)]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]

# color detection


def color_detect(img, color):
    global pub_time
    global publish_en
    global color_range_list

    if color == 'None':
        return img

    msg = Result()
    area_max = 0
    area_max_contour = 0
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(
        img_copy, size_m, interpolation=cv2.INTER_NEAREST)
    # transfer image to LAB space
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)

    if color in color_range_list:
        # TODO:define the frame_mask for the color (set the pixel inrange to 255 others to 0)
        frame_mask = cv2.inRange(frame_lab,
                                 (color_range_list[color]['min'][0],
                                  color_range_list[color]['min'][1],
                                  color_range_list[color]['min'][2]),
                                 (color_range_list[color]['max'][0],
                                  color_range_list[color]['max'][1],
                                  color_range_list[color]['max'][2]))

        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(
            cv2.MORPH_RECT, (2, 2)))          # erosion
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(
            cv2.MORPH_RECT, (2, 2)))            # dilation
        contours = cv2.findContours(
            dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]         # find all contours
        area_max_contour, area_max = getAreaMaxContour(
            contours)                                   # find the largest contours

        if area_max > 200:  # find largest area

            # TODO: find the smallest incircle center x, center y, radius from area_max_contour
            # Hint: cv2.minEnclosingCircle
            (centerx, centery), radius = cv2.minEnclosingCircle(
                area_max_contour)  # Get the smallest circumcircle
            msg.center_x = int(Misc.map(centerx, 0, size_m[0], 0, img_w))
            msg.center_y = int(Misc.map(centery, 0, size_m[1], 0, img_h))
            msg.data = int(Misc.map(radius, 0, size_m[0], 0, img_w))
            cv2.circle(img, (msg.center_x, msg.center_y),
                       msg.data+5, range_rgb[color], 2)
            publish_en = True

        if publish_en:
            if (time.time()-pub_time) >= 0.06:
                result_pub.publish(msg)  # publish
                pub_time = time.time()

            if msg.data == 0:
                publish_en = False
                result_pub.publish(msg)

    return img

# camera image callback


def image_callback(ros_image):
    global lock
    global target_type
    global target_color

    # transfer the message to image
    image = np.ndarray(shape=(ros_image.height, ros_image.width,
                       3), dtype=np.uint8, buffer=ros_image.data)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    frame_result = cv2_img
    with lock:
        if __isRunning:
            if target_type == 'apriltag':
                frame_result = apriltag_Detect(cv2_img)
            elif target_type == 'apriltag1':
                frame_result = apriltag_Detect(cv2_img, 1)
            elif target_type == 'apriltag2':
                frame_result = apriltag_Detect(cv2_img, 2)
            elif target_type == 'color':
                frame_result = color_detect(cv2_img, target_color)

    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


# init
def init():
    global publish_en
    global id_smallest
    global target_color
    global color_range_list

    publish_en = True
    id_smallest = 'None'
    target_color = 'None'
    rospy.loginfo("visual processing Init")
    # get lab range from ros param server
    color_range_list = rospy.get_param(
        '/lab_config_manager/color_range_list', {})


image_sub_st = False


def enter_func(msg):
    global lock
    global image_sub
    global image_sub_st

    rospy.loginfo("enter visual processing")
    init()
    with lock:
        if not image_sub_st:
            image_sub_st = True
            image_sub = rospy.Subscriber(
                '/usb_cam/image_raw', Image, image_callback)

    return [True, 'enter']


def exit_func(msg):
    global lock
    global image_sub
    global image_sub_st
    global __isRunning
    global target_color

    rospy.loginfo("exit visual processing")
    with lock:
        __isRunning = False
        target_color = 'None'
        try:
            if image_sub_st:
                image_sub_st = False
                image_sub.unregister()
        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running visual processing")
    with lock:
        __isRunning = True


def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running visual processing")
    with lock:
        __isRunning = False


def set_running(msg):
    global target_type
    global target_color

    rospy.loginfo("%s", msg)
    init()
    if msg.type:
        target_type = msg.type
        target_color = msg.color
        start_running()
    else:
        target_color = 'None'
        stop_running()

    return [True, 'set_running']


if __name__ == '__main__':
    # init node
    rospy.init_node('visual_processing', log_level=rospy.DEBUG)
    # communication
    image_pub = rospy.Publisher(
        '/visual_processing/image_result', Image, queue_size=1)
    # TODO: init the result_pub for publish the result
    result_pub = rospy.Publisher(
        '/visual_processing/result', Result, queue_size=1)

    enter_srv = rospy.Service('/visual_processing/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/visual_processing/exit', Trigger, exit_func)
    running_srv = rospy.Service(
        '/visual_processing/set_running', SetParam, set_running)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
