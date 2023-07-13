#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math as np
import numpy as npy
import sys
#import time

class Bot:
    def __init__(self):
        
        self.waypoints = [2.0, 0.0,5]
        self.i = 0
        
        #publisher
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        #intial velocties
        self.ivx = int(self.waypoints[0])/int(self.waypoints[2]) 
        self.ivy = int(self.waypoints[1])/int(self.waypoints[2]) 
  
        #current bot location 
        #self.x = 0
        #self.y = 0

        #PID constants
        self.kp = 0.1
        self.kd = 0.1

        #time constants
        self.t_start= rospy.get_time()
        self.time_passed = 0

def Node():
    
    bot = Bot()
    #subscriber
    bot.subscriber = rospy.Subscriber("/odom",Odometry,odom_callback)
    rospy.loginfo("Mecanum_go_to_goal_controller has started !!")
    #spin
    rospy.spin()
    

def odom_callback(odom):
    bot = Bot()
    #bot.x = odom.pose.pose.position.x
    #bot.y = odom.pose.pose.position.y

    #publish velocity for time 
    rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("starting node !!")
        #t_start =         rospy.get_time()
        time_passed = rospy.get_time() - bot.t_start
        print(time_passed)
        bot.x = odom.pose.pose.position.x
        bot.y = odom.pose.pose.position.y
        #publish velocity according to time, keep a buffer time for the bot to respond to i.e +2 sec, then publish a vel of 0 
        
        if (time_passed <= bot.waypoints[2]):
            rospy.loginfo("lessgooo1")
            cmd = Twist()
            cmd.linear.x = bot.ivx
            cmd.linear.y = bot.ivy
            cmd.linear.z=0.0
            cmd.angular.x=0.0
            cmd.angular.y=0.0
            print("current vel-", bot.ivx, bot.ivy )
            bot.pub.publish(cmd)
            rospy.sleep(1)
            #rospy.loginfo(d_v_x , d_v_y)
        elif (time_passed <= bot.waypoints[2]+15 ):
            rospy.loginfo("lessgooo2")
            cmd = Twist()
            bot.x = odom.pose.pose.position.x
            #bot.y = odom.pose.pose.position.y
            cmd.linear.x, cmd.linear.y = pid(bot.x, bot.y, bot.waypoints[0], bot.waypoints[1])
            cmd.linear.z=0.0
            cmd.angular.x=0.0
            cmd.angular.y=0.0
            print("current vel-", cmd.linear.x, cmd.linear.y )
            bot.pub.publish(cmd)
            rospy.sleep(0.1)
            #rospy.loginfo(cmd.linear.x , cmd.linear.y)        
        else:
            cmd = Twist()
            rospy.loginfo("lessgooo3")
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z= 0.0
            cmd.angular.x= 0.0
            cmd.angular.y= 0.0
            print("current vel-", cmd.linear.x, cmd.linear.y )
            bot.pub.publish(cmd)
            #sys.exit()
            rospy.sleep(10)
        #sys.exit()
        def pid (x, y, px, py):
            bot = Bot()
            print ("pid -", x, y, px, py)
            vel_x , vel_y = bot.kp*(px-x) , bot.kp*(py - y)
            return vel_x, vel_y
            

    #publish velocity for pid
    #stop 

    #pass

"""def pid (x, y, px, py):
    bot = Bot()
    print ("pid -", x, y, px, py)
    vel_x , vel_y = bot.kp*(px-x) , bot.kp*(py - y)
    return vel_x, vel_y"""
    #pass

def main():
    rospy.init_node('go_to_goal', anonymous=True)
    print("Node spinning will start")
    Node()
    #sys.exit()
    print("Node spinning will is ended") 
    #pass

if __name__ == "__main__":
        main()
        #sys.exit()





        