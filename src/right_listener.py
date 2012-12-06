#!/usr/bin/env python

import roslib; roslib.load_manifest('laser_listener')
import rospy
import math
import sys
from detect_steps import detectSteps

from std_msgs.msg import *
from sensor_msgs.msg import LaserScan

def callBackFunction(data):
    pub = rospy.Publisher('step_detect_right', Float32)
    detectSteps(pub,data)

def main():
    rospy.init_node('right_listener', anonymous=False)
    rospy.Subscriber("kerbDetectRight", LaserScan, callBackFunction)
    rospy.spin()

if __name__ == '__main__':
	main()
