#!/usr/bin/env python

import roslib; roslib.load_manifest('laser_listener')
import rospy
import math
import sys
from detect_steps import detectSteps

from std_msgs.msg import *
from sensor_msgs.msg import LaserScan

def callBackFunction(data):
    pub = rospy.Publisher('step_detect_left', Float32)
    detectSteps(pub,data)

def main():
    #pub = rospy.Publisher('step_detect_left', Float32)
    rospy.init_node('left_listener', anonymous=False)
    rospy.Subscriber("kerbDetectLeft", LaserScan, callBackFunction)
    rospy.spin()

if __name__ == '__main__':
	main()
#    print (sys.argv)
#    if len(sys.argv) <= 4:
#	rospy.loginfo("Please add nodeName, topic to listen to and topic to publish to as program arguments") 
#	sys.exit()
#    else:
#   	print "%s,%s,%s"%(str((sys.argv[1])),str((sys.argv[2])),str((sys.argv[3])))
#    	main(str((sys.argv[1])), str((sys.argv[2])), str((sys.argv[3])))

