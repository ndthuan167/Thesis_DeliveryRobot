#!/usr/bin/env python

import rospy
from utils.msg import DiffVel

import math
import numpy
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('test_actuator')
    pub = rospy.Publisher('diff_motor/vel', DiffVel, queue_size=5)

    while(1):
        key = getKey()

        if key == ' ':
            msg = DiffVel()
            msg.left_vel = 4.1887
            msg.right_vel = 4.1887
            pub.publish(msg)

        elif key == '\x03':
            break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

