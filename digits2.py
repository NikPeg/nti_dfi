from __future__ import print_function
import math
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from clover import srv
from threading import Thread
from clover.srv import SetLEDEffect



# inits
rospy.init_node('flight')
bridge = CvBridge()

# proxys

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# pubs
digits_debug = rospy.Publisher("/digits_debug", Image)


def check_digits(data):
    global color_counter
    global cap  # var for waiting the capture
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')  # get frame
    model=cv.ml.KNearest_load('model.yaml')
    im = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    out = np.zeros(im.shape,np.uint8)
    gray = cv.cvtColor(im,cv.COLOR_BGR2GRAY)
    thresh = cv.adaptiveThreshold(gray,255,1,1,11,2)
    # contours, hierarchy = cv.findContours(thresh,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    string = ''
    for cnt in contours:
        if cv.contourArea(cnt)>50:
            [x,y,w,h] = cv.boundingRect(cnt)
            if  h>28:
                cv.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
                roi = thresh[y:y+h,x:x+w]
                roismall = cv.resize(roi,(10,10))
                roismall = roismall.reshape((1,100))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k = 10)
                string = str(int((results[0][0])))
                cv.putText(out,string,(x,y+h),0,1,(0,255,0))
                #shows the information
                if string != 0:
                    cv.imshow('im',im)
                    cv.imshow('out',out)
                    dronpoints[int(string)] = (i, j)
    color_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))  # publish to topic (for web-video-server)

image_sub = rospy.Subscriber('main_camera/image_raw', Image, check_digits, queue_size=1)  # get capture
rospy.spin()