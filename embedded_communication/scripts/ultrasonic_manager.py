#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from embedded_communication.msg import UltraSonic

ultrasonic_sensors = rospy.get_param("/robot/ultrasonic/sensors")
ultra_min = rospy.get_param("/robot/ultrasonic/ultrasonic_min")
ultra_max = rospy.get_param("/robot/ultrasonic/ultrasonic_max")

## Threshold values
clear_threshold = rospy.get_param("/robot/ultrasonic/clear_threshold")
mark_threshold = rospy.get_param("/robot/ultrasonic/mark_threshold") * 100


## General sensors Based Information in Range MSG type for ultrasonic
UltraMsg = Range()
UltraMsg.field_of_view = 1.0
UltraMsg.radiation_type = 0
UltraMsg.min_range = 0 
UltraMsg.max_range = 5


## Adding TF Frames for All the ultrasonic sensors
frames = list()
for i in range(0,len(ultrasonic_sensors)):
    frames.append("ultra_"+str(i+1))

def CalcMaxRangeWithThreshold(value):
    global mark_threshold
    max_value = (value/mark_threshold)*100/100
    return max_value

def UltrasonicCB(sensors):
    ultra_list = sensors.distances
    ultra_values = list()
    for i in ultrasonic_sensors:
        ultra_values.append(ultra_list[i])
    for i in range(0,len(ultra_values)):
        if ultra_values[i] < ultra_max and ultra_values[i]> ultra_min:
            rospy.logwarn("Ultrasonic Detected at  ultrasonic_sensors %s",i)
            OutData = UltraMsg
            OutData.header.frame_id = frames[ultrasonic_sensors.index(i)]
            OutData.header.stamp = rospy.Time.now()
            OutData.range = ultra_values[i]/100
            OutData.max_range = CalcMaxRangeWithThreshold(ultra_values[i])
            globals()['ultrapub%s' %ultrasonic_sensors.index(i)].publish(OutData)
        else:
            OutData = UltraMsg
            OutData.header.frame_id = frames[ultrasonic_sensors.index(i)]
            OutData.header.stamp = rospy.Time.now()
            OutData.range = 5 #CalcMaxRangeWithThreshold(ultra_values[i])
            OutData.max_range = 5 #CalcMaxRangeWithThreshold(ultra_values[i])
            globals()['ultrapub%s' %ultrasonic_sensors.index(i)].publish(OutData)


## Main Function
if __name__ == "__main__":
    rospy.init_node("ultrasonic_manager")
    rospy.loginfo("Ultrasonic manager node started")  
    for i in range(0,len(ultrasonic_sensors)):
        globals()['ultrapub%s' %i] = rospy.Publisher("/ultra"+str(i+1),Range,queue_size=1)
    rospy.Subscriber("/ultrasonic_data",UltraSonic,UltrasonicCB)
    rospy.spin()
