#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist,Vector3,Point
from nav_msgs.msg import Odometry  
import math as np
import numpy as npy

class Data:
   def __init__(self):
        self.x = 0.0 
        self.y = 0.0 
        self.vel_x = 0.0 
        self.vel_y = 0.0
        self.curr_yaw = 0.0
        self.prev_des_x = 0.0
        self.prev_des_y = 0.0
        # PID gains for error in X direction
        self.kp_v_x=1.79
        self.kd_v_x=0.68
        self.ki_v_x=0.00018
        # PID gains for error in Y direction
        self.kp_v_y=1.68
        self.kd_v_y=0.59
        self.ki_v_y=0.00018
        #PID gains for error in W
        self.kp_w= 0.8 
        self.kd_w=0.0   
        self.ki_w=0.0   
        # in PID e_x_int stands for sum of errors in X
        #        e_x_old to store previous error in X 
        self.e_x_int=0
        self.e_x_old=0
        # in PID e_y_int stands for sum of errors in Y
        #        e_y_old to store previous error in Y 
        self.e_y_int=0
        self.e_y_old=0

        self.e_orient_int=0
        self.e_orient_old=0

        self.e_y_diff = 0
        self.e_x_diff = 0
        self.e_orient_diff=0
        self.publish_=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        

def node_start():
    global data
    data = Data() 
      
    
    subscriber_waypoint=rospy.Subscriber("/waypoints_to_follow",Vector3,go_to_goal)
    subscriber_=rospy.Subscriber("/odom",Odometry,odometry_data_callback)
    rospy.loginfo("Mecanum_Traj_controller has started !!")

    rospy.spin()

def quat_to_euler(x,y,z,w) :
      
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.atan2(t3, t4) # to get udate of bots orientation from topic orient_euler
   
    return yaw_z


def odometry_data_callback(odom):  #gets executed everytime the msg on odom arrives to update the current x and y
    # global variables x and y        
    data.x = odom.pose.pose.position.x 
    data.y = odom.pose.pose.position.y
    data.vel_x = odom.twist.twist.linear.x
    data.vel_y = odom.twist.twist.linear.y
    data.curr_yaw = quat_to_euler(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w)
    # rospy.loginfo("odom data x: "+str(data.x)+" y: "+str(data.y)+" yaw: "+str(data.curr_yaw))


def go_to_goal(waypoint):
        # rospy.loginfo("Waypoints start recieving !!" + " " + str(waypoint.x) + "," + str(waypoint.y))
        des_x = waypoint.x 
        des_y = waypoint.y
        if des_x != data.prev_des_x and des_y != data.prev_des_y :
           data.e_x_diff = 0
           data.e_y_diff = 0
        #    data.e_orient_diff = 0
           data.e_x_int = 0
           data.e_y_int = 0
           data.e_x_old = 0
           data.e_y_old = 0
         #   data.e_orient_old = 0
        # e_x : error in X
        # e_y : error in Y
        # e_orient : error in Orientation
        e_x=des_x-data.x
        e_y=des_y-data.y
        rospy.loginfo("odom data x: "+str(data.x)+" y: "+str(data.y)+" yaw: "+str(data.curr_yaw))
        # rospy.loginfo("Errors   X :" + " " + str(e_x) + ", Y: " + str(e_y))
        angle_diff=np.atan2((des_y-data.y),(des_x-data.x))
        e_orient=angle_diff- data.curr_yaw
        # rospy.loginfo("x : " + str(data.x) + " y :" + str(data.y))
           
        u_w = data.kp_w*e_orient+data.kd_w*data.e_orient_diff+data.ki_w*data.e_orient_int
        u_x = data.kp_v_x*e_x+data.kd_v_x*data.e_x_diff+data.ki_v_x*data.e_x_int
        u_y = data.kp_v_y*e_y+data.kd_v_y*data.e_y_diff+data.ki_v_y*data.e_y_int 
        
        data.e_x_int+=e_x
        data.e_y_int+=e_y
        data.e_x_old=e_x
        data.e_y_old=e_y
        data.e_orient_int+=e_orient
        data.e_orient_old=e_orient
        # e_x_diff = calculating difference in error in X and Y
        data.e_x_diff=e_x-data.e_x_old
        data.e_y_diff=e_y-data.e_y_old
        data.e_orient_diff=e_orient-data.e_orient_old
        # limiting conditions to limit obtained velocities via PID 
        if u_x >= 0.5 :
            u_x = 0.5
        elif u_x <= -0.5 :
            u_x = -0.5

        if u_y >= 0.5 :
            u_y = 0.5
        elif u_y <= -0.5 :
            u_y = -0.5

        if u_w >= 1 :
                u_w = 1
        elif u_w <= -1 :
                u_w = -1

        # Frame transformations from Global to Body frame
        v_x = (u_x * np.cos(data.curr_yaw)) + (u_y * np.sin(data.curr_yaw))
        v_y = - (u_x * np.sin(data.curr_yaw)) + (u_y * np.cos(data.curr_yaw))  
        v_w = 0.5 #u_w
        # rospy.loginfo("writitng calculated cmds to bot ")
        # publishing Calculated velocities
        cmd=Twist()
        cmd.linear.x= float(v_x)
        cmd.linear.y= float(v_y)
        cmd.linear.z= 0.0
        cmd.angular.x= 0.0
        cmd.angular.y= 0.0
        # cmd.angular.z= float(v_w)
        data.publish_.publish(cmd)
        data.prev_des_x = des_x
        data.prev_des_y = des_y
        # Sleep for a short duration to control the loop rate
        # time.sleep(0.1)

def main(args=None):
    rospy.init_node('mecanum_traj_controller', anonymous=True)
    print("Node spinning will start")
    node_start()
    print("Node spinning will is ended") 
     
if __name__ == "__main__":
        main()