#!/usr/bin/env python

import roslib; roslib.load_manifest('laser_listener')
import rospy
import math
import sys

from std_msgs.msg import *
from sensor_msgs.msg import LaserScan

pub = None

def detectSteps(data):
    global pub
    # Create publisher
    #pub = rospy.Publisher(publisherTopic, Float32)

    # Height of step (metres)
    STEP_THRESH = 0.06;

    # Distance to look at beyond the detected step (metres)
    LOOK_AHEAD = 0.05;
    
    # Distance value to output if no step is detected
    MAX_DISTANCE = 5000000

    # Number of samples in input frame
    no_samples = len(data.ranges);

    # Initialise height and distance lists to zero
    height   = [0]*no_samples;
    distance = [0]*no_samples

    # Calculate vertical and horizontal projections of all input range values
    for i in range(no_samples):
        height[i]   = data.ranges[i]*math.cos(data.angle_increment*i);
        distance[i] = data.ranges[i]*math.sin(data.angle_increment*i);

    # Set the height of the laser from the ground as the first height sample (vertically downwards)
    laser_height = height[0]

    # Initialise final index
    final_index = no_samples-1;
    
    # Loop over samples from furthest to closest measuring the difference
    # between the furthest and the current. Stop when the distance is greater
    # than LOOK_AHEAD, setting the final valid index for step detection.
    for i in range(2,no_samples):
        if distance[no_samples-1] - distance[no_samples-i] > LOOK_AHEAD:
            final_index = no_samples-i;
            break;

    # Loop over valid input samples
    for i in range(1,final_index):
    
        # Break out of loop if distance less than half of the laser height threshold
        if height[i] < laser_height/2:
            break;
    
        # Calculate the difference between consecutive samples
        #laser_diff = math.fabs(height[i] - height[i-1]);
        laser_diff = height[i] - height[i-1];

        # Only consider differences greater than the step threshold
        if laser_diff > STEP_THRESH:
        
            # Propose step as detected for current sample
            step = True;
            
            # Loop over samples within LOOK_AHEAD metres of the detected step. Step detected only 
            # remains true if all samples within LOOK_AHEAD are above the step threshold
            for j in range(i,no_samples-1):
                if distance[j] - distance[i-1] < LOOK_AHEAD and height[j] < height[i-1] + STEP_THRESH:
                    step = False;

            # Publish distance to step if still determined as valid
            if step:
                #print "Step detected: " + str(distance[i-1]) + " metres."
                pub.publish(Float32(distance[i-1]))
                break;
                
        # If no step has been detected within the valid range of input samples, output MAX_DISTANCE
        # REMOVE ELIF TO STOP PUBLISHING WHEN NO STEP DETECTED
        elif i == final_index-1:
            pub.publish(Float32(MAX_DISTANCE))       

def main(nodeName, hokuyoTopicToListenTo, topicToPublish):
    pub = rospy.Publisher(topicToPublish, Float32)
    rospy.init_node(nodeName, anonymous=False)
    rospy.Subscriber(hokuyoTopicToListenTo, LaserScan, detectSteps)
    rospy.spin()

if __name__ == '__main__':
    print (sys.argv)
#    if len(sys.argv) >= 4:
#	rospy.loginfo("Please add nodeName, topic to listen to and topic to publish to as program arguments") 
#	sys.exit()
#    else:
    print "%s,%s,%s"%(str((sys.argv[1])),str((sys.argv[2])),str((sys.argv[3])))
    main(str((sys.argv[1])), str((sys.argv[2])), str((sys.argv[3])))

