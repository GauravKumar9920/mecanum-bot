#!/usr/bin/env python2
import csv
import rospy
import tf.transformations
from geometry_msgs.msg import Twist

def shutdown_callback():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	vel.linear.x=0
	vel.linear.y=0
	pub.publish(Twist())
    
            
if __name__ == '__main__':
    
    print("init")
    with open('/home/dotkat012/ros/generic_ws/src/robot-alog/alog_control/scripts/vel_x_data.csv') as f:
    	reader = csv.reader(f)
	print(type(reader))
    	data = list(reader)
        print(data[0][0])
	data_n= [float(p[0]) for p in data]
        data_x=(data_n[100:120])
	#print(data_x)
    with open('/home/dotkat012/ros/generic_ws/src/robot-alog/alog_control/scripts/vel_y_data.csv') as g:
    	reader = csv.reader(g)
	print(type(reader))
    	data = list(reader)
        print(data[0][0])
	data_n1= [float(p[0]) for p in data]
	data_y=(data_n1[100:120])
    #with open('/home/dotkat012/ros/generic_ws/src/robot-alog/alog_control/scripts/vel_r_data.csv') as h:
    #	reader = csv.reader(h)
	#print(type(reader))
    #	data = list(reader)
     #   print(data[0][0])
	#data_r= [float(p[0]) for p in data]
    
    print("data read")
   
    #rospy.on_shutdown(shutdown_callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_array')
    rate= rospy.Rate(1)
    cmd_vel_x_array = 10*data_x
    cmd_vel_y_array = 10*data_y
   # cmd_vel_r_array = 10*data_r 
    vel =Twist()
    while not rospy.is_shutdown():
        for i in range(len(cmd_vel_x_array)):
            vel.linear.x = cmd_vel_x_array[i]
	    vel.linear.y = cmd_vel_y_array[i]
         #   vel.angular.z = cmd_vel_r_array[i]
            print("publishing vel")
            pub.publish(vel)
            i=i+1
	

    
   
    
