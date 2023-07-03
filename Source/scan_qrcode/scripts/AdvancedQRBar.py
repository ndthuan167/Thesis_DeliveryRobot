#!/usr/bin/env python3

import imp
from math import fabs
from multiprocessing import Condition
import cv2
import numpy as np
from numpy import rate
import rospy
from utils.msg import Complete
from utils.msg import string_of_qrcode
from pyzbar.pyzbar import decode
from utils.msg import WaypointList
from utils.msg import mode_indoor
from utils.msg import Odometry
from utils.msg import mode_indoor

x_A = -95.9571046008 
y_A = -15.0817014185 

x_B = -96.5417280523 
y_B = -24.1511336597 

lat_end_waypoint_set = 107746320.00 
lon_end_waypoint_set = 1066596096.00

lat_end_waypoint = 0.0
lon_end_waypoint = 0.0

a = 0.0
mod = 0.0

def CallbackComplete():
    rospy.init_node('ScanQR', anonymous=True)
    rospy.Subscriber("/Complete", Complete , SetQR)
    rospy.Subscriber("/odom_1", Odometry, CallOdom )
    rospy.Subscriber("/mission/item_list", WaypointList, Callwaypoint)
    rospy.Subscriber("/mode_indoor", mode_indoor, CallbackMode)

    rospy.spin()

def CallbackMode(mode):
    global mod, set_complete_indoor
    mod = mode.mode_indoor
    set_complete_indoor = mode.set_complete_indoor

def CallOdom(pos):
    x_current = pos.position.x 
    y_current = pos.position.y

def Callwaypoint(way):
    global a
    global lat_end_waypoint
    lat_end_waypoint = way.waypoints.pop().x_lat
    lon_end_waypoint = way.waypoints.pop().y_long
    a = lat_end_waypoint-lat_end_waypoint_set
    print(a)


def SetQR(data):
    global Data, pub_complete, a
    Data = None
    setQR = data.complete
    setQR_indoor = data.complete_indoor 
    if a != 0.0 :
        if ( setQR == True) :
            cap = cv2.VideoCapture(0)
            cap.set(3, 1000)
            cap.set(4, 1000)
            used_code = []
            camera = True
            while camera:
                success, frame = cap.read()
                if Data != None:
                    cv2.destroyAllWindows()
                    cap.release()
                for code in decode(frame):
                    Data = code.data.decode('utf-8')
                    # if Data not in used_code:
                    #     myColor = (0, 255, 0)
                    #     used_code.append(Data)
                    # else:
                    #     myColor = (0, 0, 255)
                    myColor = (255, 0, 255)
                    pts = np.array([code.polygon], np.int32)
                    pts = pts.reshape((-1,1,2))
                    cv2.polylines(frame, [pts], True, myColor, 5)
                    pts2 = code.rect
                    cv2.putText(frame, Data, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, myColor, 2)

                    pub_complete.publish(Data)
                cv2.imshow('QR-Scan', frame)
                cv2.waitKey(1)
        
    if a == 0.0:
        if ( setQR_indoor == True & set_complete_indoor == True) :
            cap = cv2.VideoCapture(3)
            cap.set(3, 1000)
            cap.set(4, 1000)
            used_code = []
            camera = True
            while camera:
                success, frame = cap.read()
                if Data != None:
                    cv2.destroyAllWindows()
                    cap.release()
                for code in decode(frame):
                    Data = code.data.decode('utf-8')
                    # if Data not in used_code:
                    #     myColor = (0, 255, 0)
                    #     used_code.append(Data)
                    # else:
                    #     myColor = (0, 0, 255)
                    myColor = (255, 0, 255)
                    pts = np.array([code.polygon], np.int32)
                    pts = pts.reshape((-1,1,2))
                    cv2.polylines(frame, [pts], True, myColor, 5)
                    pts2 = code.rect
                    cv2.putText(frame, Data, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, myColor, 2)

                    pub_complete.publish(Data)
                cv2.imshow('Testing-QR-Scan', frame)
                cv2.waitKey(1)            


if __name__ == '__main__':
    pub_complete = rospy.Publisher("/String_of_QRCode", string_of_qrcode, queue_size=10 )
    CallbackComplete()
    
 