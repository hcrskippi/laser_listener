import rospy
import math
import sys

from std_msgs.msg import *
from sensor_msgs.msg import LaserScan

def detectSteps(pub, data):
    # Height of step (metres)
    STEP_THRESH = 0.06;

    # Distance to look at beyond the detected step (metres)
    LOOK_AHEAD = 0.04;

    # Distance value to output if no step is detected
    MAX_DISTANCE = 5000000

    # Number of samples in input frame
    no_samples = len(data.ranges);

    # Initialise height and distance lists to zero
    height   = [0.0]*no_samples;
    distance = [0.0]*no_samples

    # Calculate vertical and horizontal projections of all input range values
    for i in range(no_samples):
        sample = data.ranges[i]
        if sample < 0.01:
            sample  = 100000000
        height[i]   = sample*math.cos(data.angle_increment*i);
        distance[i] = sample*math.sin(data.angle_increment*i);
	
    # Set the height of the laser from the ground as the first height sample (vertically downwards
    laser_height = height[0]

 #   for i in range(1,no_samples):
#	if height[i] < laser_height:
#	    height[i]   = 10
#	    distance[i] = 10

    # Initialise final index
    final_index = no_samples-1;

    # Loop over samples from furthest to closest measuring the difference
    # between the furthest and the current. Stop when the distance is greater
    # than LOOK_AHEAD, setting the final valid index for step detection.
    last_valid_sample=1;
    for i in range(2,no_samples):
	if distance[no_samples-(i-1)] > 0.1 and last_valid_sample == 1:
	    last_valid_sample = no_samples-(i-1)
        if distance[last_valid_sample] - distance[no_samples-i] > LOOK_AHEAD and last_valid_sample >1:
            final_index = no_samples-i;
            break;

    # Loop over valid input samples
    for i in range(1,final_index):

        # Break out of loop if distance less than half of the laser height threshold
        if height[i] < laser_height/2.0:
             pub.publish(Float32(MAX_DISTANCE-1))
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
            for j in range(i,no_samples):
                if distance[j] - distance[i-1] < LOOK_AHEAD and height[j] < height[i-1] + STEP_THRESH and distance[j]>0.1:
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

