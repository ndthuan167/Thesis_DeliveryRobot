#!/usr/bin/env python3

import rospy
from utils.msg import DiffVel
from utils.srv import CommandLong, CommandLongResponse

import struct, time, can
import sys, termios
from can.interfaces.robotell import robotellBus
    
def get_int(ba):
    return int.from_bytes(ba, 'big')
    
def send_one(bus,controll_onoff):
    global vel_left , vel_right

    global motorsLocked
      
    values = [abs(vel_left), abs(vel_right)]
    
    
    ba = bytearray(struct.pack('>f', values[0]))
    ba1 = bytearray(struct.pack('>f', values[1]))
    payload = [get_int(ba[0:1]),get_int(ba[1:2]),get_int(ba[2:3]),get_int(ba[3:4]),get_int(ba1[0:1]),get_int(ba1[1:2]),get_int(ba1[2:3]),get_int(ba1[3:4])]
    frame = can.Message(arbitration_id=0x111, data=payload, is_extended_id=False)
    try:
        bus.send(frame)
        print(frame)
    except can.CanError:
        print("Message not sent.")

    time.sleep(0.1)
    
    
def onMotorsCmdCallBack(msg):
    global bus, vel_left , vel_right,motorsLocked
    print(msg)
    if 1:# motorsLocked == False:
        vel_left = msg.left_vel
        vel_right = msg.right_vel

    # else :
    #     vel_left = 0.0
    #     vel_right = 0.0

    send_one(bus, 1.0)
    
def onSetArmingCallBack(req):
    global motorsLocked
    if req.param1 == 1.0:
        motorsLocked = False
        send_one(bus, 1.0)
        rospy.loginfo('Motors unlocked.')
    elif req.param1 == 0.0:
        motorsLocked = True
        send_one(bus, 0.0)
        rospy.loginfo('Motors locked.')
    res = CommandLongResponse(True, 0)
    return res

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('can_transceiver')
    enabled = True
    port = '/dev/ttyUSB0'
    # port1 = '/dev/ttyUSB1'
    baudrate = 115200

    subMotorsCmd = rospy.Subscriber('diff_motor/vel', DiffVel, onMotorsCmdCallBack)
    resSetArming = rospy.Service('command/set_arming', CommandLong, onSetArmingCallBack)
    bus = robotellBus(channel=port, ttyBaudrate=baudrate)

    rospy.spin()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
