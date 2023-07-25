#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from embedded_communication.msg import Cliff

cliff_sensors = rospy.get_param("/robot/cliff/sensors")
ground_values = rospy.get_param("/robot/cliff/ground_values")
tolerance = rospy.get_param("/robot/cliff/tolerance")

## Threshold values
clear_threshold = rospy.get_param("/robot/cliff/clear_threshold")
mark_threshold = rospy.get_param("/robot/cliff/mark_threshold") * 100


## General sensors Based Information in Range MSG type for ultrasonic
CliffMsg = Range()
CliffMsg.field_of_view = 0.698132
CliffMsg.radiation_type = 0
CliffMsg.min_range = 0 
CliffMsg.max_range = 5

## Adding TF Frames for All the Cliff sensors
frames = list()
for i in range(0,len(cliff_sensors)):
    frames.append("cliff_"+str(i+1))

def CalcMaxRangeWithThreshold(value):
    global mark_threshold
    max_value = (value/mark_threshold)*100/100
    return max_value

def CliffCB(sensors):
    cliff_list = sensors.cliff
    cliff_values = list()
    for i in cliff_sensors:
        cliff_values.append(cliff_list[i])
    for i in range(0,len(cliff_values)):
        if cliff_values[i]>(ground_values[i]+tolerance):
            rospy.logwarn("Cliff Detected at front cliff_sensors %s",i)
            OutData = CliffMsg
            OutData.header.frame_id = frames[cliff_sensors.index(i)]
            OutData.header.stamp = rospy.Time.now()
            OutData.range = cliff_values[i]/100
            OutData.max_range = CalcMaxRangeWithThreshold(cliff_values[i])
            globals()['cliffpub%s' %cliff_sensors.index(i)].publish(OutData)
        else:
            pass

## Main Function
if __name__ == "__main__":
    rospy.init_node("cliff_manager")
    rospy.loginfo("Cliff manager node started")  
    for i in range(0,len(cliff_sensors)):
        globals()['cliffpub%s' %i] = rospy.Publisher("/cliff"+str(i+1),Range,queue_size=1)
    rospy.Subscriber("/cliff_data",Cliff,CliffCB)
    rospy.spin()
