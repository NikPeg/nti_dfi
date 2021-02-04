from __future__ import print_function
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
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
color_debug = rospy.Publisher("/color_debug", Image)
def check_temp(data):
    global color_counter
    global cap  # var for waiting the capture
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')[50:160, 100:220]  # get frame
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # get binarized images in each color
    red = cv.inRange(hsv, (165, 70, 158), (255, 209, 255))
    yellow = cv.inRange(hsv, (10, 80, 88), (49, 220, 225))
    green = cv.inRange(hsv, (65, 84, 79), (70, 209, 187))
    blue = cv.inRange(hsv, (94, 88, 63), (134, 255, 255))
    black = cv.inRange(hsv, (0, 0, 0), (255, 255, 115))
    
    # count non-zero pixels
    color = {'r': cv.countNonZero(red),
             'y': cv.countNonZero(yellow),
             'g': cv.countNonZero(green),
             'b': cv.countNonZero(blue),
             'bl': cv.countNonZero(black)}

      
    temperature = max(color, key=color.get)  # get max key
    #print(n,color, '     ', temperature)
    color_counter[temperature] += 1 
   

   # draw circle in centor of colored spot (only need color)
    try:
        if temperature == 'r':
            moments = cv.moments(red, 1)  # get moments for find the center
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 0, 255), -1)  # draw
            color_counter['r'] = color_counter.get('r', 0) + 1
        if temperature == 'y':
            moments = cv.moments(yellow, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 255, 255), -1)
            color_counter['y'] = color_counter.get('y', 0) + 1
        if temperature == 'g':
            moments = cv.moments(green, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 255, 0), -1)
            color_counter['g'] = color_counter.get('g', 0) + 1
        if temperature == 'b':
            moments = cv.moments(blue, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (255, 0, 0), -1)
            color_counter['b'] = color_counter.get('b', 0) + 1
        if temperature == 'bl':
            moments = cv.moments(black, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 0, 0), -1)
            




    except ZeroDivisionError:
        print('zero')

    color_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))  # publish to topic (for web-video-server)
    

    image_sub.unregister()
    cap = True










coords = {1: [0, 4.5, 1],
          2: [0.45, 4.5, 1],
          3: [0.9, 4.5, 1],
          4: [1.35, 4.5, 1],
          5: [1.8 , 4.5, 1],
          6: [2.25, 4.5, 1],
          7: [2.7, 4.5, 1],
          8: [3.15, 4.5, 1],
          9: [3.6, 4.5, 1],
          10: [3.6, 4.95, 1],
          11: [3.15, 4.95, 1],
          12: [2.7, 4.95, 1],
          13: [2.25, 4.95, 1],
          14: [1.8, 4.95, 1],
          15: [1.35, 4.95, 1],
          16: [0.9, 4.95],
          17: [0.45, 4.95, 1],
          18: [0, 4.95, 1],
          19: [0, 5.4, 1],
          20: [0.45, 5.4, 1],
          21: [0.9, 5.4, 1],
          22: [1.35, 5.4],
          23: [1.8, 5.4, 1],
          24: [2.25, 5.4, 1],
          25: [2.7, 5.4, 1],
          26: [3.15, 5.4, 1],
          27: [3.6, 5.4, 1]}
          

path = [1, 2, 3, 4, 5, 6, 7, 8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27]
navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(6)
color_counter = {'r': 0, 'b': 0, 'g': 0, 'y': 0, 'bl': 0}

for n in path:
    cap = False
    print()
    #print('flying', n, coords[n])
    navigate(x=coords[n][0], y=coords[n][1], z=coords[n][2], frame_id='aruco_map')  # go to point
    rospy.sleep(12)
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, check_temp, queue_size=1)  # get capture
    while not cap:  # wait the capture
        rospy.sleep(1)
    rospy.sleep(3)

sum = color_counter.get('y', 0) + color_counter.get('g', 0) + color_counter.get('b', 0) + color_counter.get('r', 0)
print("Balance {0} cargo\nType 0: {1} cargo\nType 1: {2} cargo\nType 2: {3} cargo\nType 3: {4} cargo".format(sum, color_counter.get('y', 0), color_counter.get('g', 0), color_counter.get('b', 0), color_counter.get('r', 0)))


#######################################################################


model=cv2.ml.KNearest_load('model.yaml')

def find_digit():
    im = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')[50:160, 150:220]
    out = np.zeros(im.shape,np.uint8)
    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(gray,255,1,1,11,2)

    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    digit_count = {0: 0, 1: 0, 2: 0, 3: 0}
    for cnt in contours:
        if cv2.contourArea(cnt)>50:
            [x,y,w,h] = cv2.boundingRect(cnt)
            if  h>28:
                cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
                roi = thresh[y:y+h,x:x+w]
                roismall = cv2.resize(roi,(10,10))
                roismall = roismall.reshape((1,100))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k = 10)
                digit_count[int((results[0][0]))] += 1
    if max(digit_count, key=digit_count.get) == 0:
        return None
    return max(digit_count, key=digit_count.get)

def check_for_white():global color_counter
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')[50:160, 150:220]
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # get binarized images in each color
    red = cv.inRange(hsv, (165, 70, 158), (255, 209, 255))
    yellow = cv.inRange(hsv, (10, 80, 88), (49, 220, 225))
    green = cv.inRange(hsv, (65, 84, 79), (70, 209, 187))
    blue = cv.inRange(hsv, (94, 88, 63), (134, 255, 255))
    black = cv.inRange(hsv, (0, 0, 0), (255, 255, 115))
    white = cv.inRange(hsv, (0, 100, 100), (20, 255, 255))
    
    # count non-zero pixels
    color = {'r': cv.countNonZero(red),
             'y': cv.countNonZero(yellow),
             'g': cv.countNonZero(green),
             'b': cv.countNonZero(blue),
             'bl': cv.countNonZero(black),
             'w': cv.countNonZero(white)}
    temperature = max(color, key=color.get)
    return temperature == 'w'
    

dronpoints = {0: 0, 1: 0, 2: 0, 3: 0}
for i in range(0, 5.5, 0.5):
    for j in range(0, 5.5, 0.5):
        navigate(x=i, y=j, z=1, speed=0.5, frame_id='aruco_map', auto_arm=True)
        if check_for_white():
            fd = find_digit()
            if fd is not None:
                dronpoints[fd] = (i, j)

land()
colors = ((255, 255, 0), (0, 255, 0), (0, 0, 255), (255, 0, 0))
point_type = ('products', 'clothes', 'fragile packaging', 'correspondence')
for i in dronpoints.keys():
    coord = dronpoints[i]
    navigate(x=coord[0], y=coord[1], z=2, speed=1, frame_id='aruco_map')
    rospy.sleep(5)
    land()
    set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect) # define proxy $
    set_effect(effect='fade', r=colors[i][0], g=colors[i][1],b=colors[i][2])
    print("D{0}_delivered {1}".Format(i, point_type[i]))




