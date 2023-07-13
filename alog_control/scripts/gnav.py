#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math as np
import numpy as npy
#import time

def node():
    global x , y
    x = 0
    y = 0

    waypoints = [2.5, 0.0,5]
    rate=rospy.Rate(10)
    publisher =rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    subscriber =rospy.Subscriber("/odom",Odometry, odometry_data_callback)
    #rospy.spin()
    #default set velocity
    d_v_x = int(waypoints[0])/int(waypoints[2])   
    d_v_y = int(waypoints[1])/int(waypoints[2])
    rospy.Rate(10)
    t_start =  rospy.get_time()
    while not rospy.is_shutdown():
        rospy.loginfo("starting node !!")
        #t_start =         rospy.get_time()
        time_passed = rospy.get_time() - t_start
        print(time_passed)
        #publish velocity according to time, keep a buffer time for the bot to respond to i.e +2 sec, then publish a vel of 0 
        
        if (time_passed <= float(waypoints[2])):
             rospy.loginfo("lessgooo!!!")
             cmd = Twist()
             cmd.linear.x = d_v_x
             cmd.linear.y = d_v_y
             cmd.linear.z=0.0
             cmd.angular.x=0.0
             cmd.angular.y=0.0
             publisher.publish(cmd)
             print(d_v_x, d_v_y)
             rospy.sleep(1)
             #rospy.loginfo(d_v_x , d_v_y)
        elif (time_passed <= 7):
             rospy.loginfo("lessgooo1")
             cmd = Twist()
             cmd.linear.x, cmd.linear.y = pid(d_v_x, d_v_y, waypoints[0], waypoints[1])
             #cmd.linear.y = pid(d_v_y)
             cmd.linear.z=0.0
             cmd.angular.x=0.0
             cmd.angular.y=0.0
             publisher.publish(cmd)
             rospy.sleep(0.1)

             #rospy.loginfo(cmd.linear.x , cmd.linear.y)        
        else:
             cmd = Twist()
             cmd.linear.x = 0.0
             cmd.linear.y = 0.0
             cmd.linear.z= 0.0
             cmd.angular.x= 0.0
             cmd.angular.y= 0.0
             publisher.publish(cmd)
             #rospy.loginfo(cmd.linear.x , cmd.linear.y)
        #rospy.spin()



def odometry_data_callback(odom):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    #return (i_x, i_y)

def pid(vx , vy, px, py):
     kp = 1
     kd = 1
     #c_x, c_y = odometry_data_callback()
     p_v_x , p_v_y = kp(px - x) , kp(py - y)
     
     return (p_v_x, p_v_y)

def main(args=None):
    rospy.init_node('go_to_goal_controller', anonymous=True)
    print("Node spinning will start")
    node()
    #rospy.spin()
    print("Node spinning will is ended") 
     
if __name__ == "__main__":
        main()