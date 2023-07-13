#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math as np
import numpy as npy



def node_start():

    publish_waypoints = rospy.Publisher('/waypoints_to_follow',Vector3,queue_size=10)
    publish_plot_pts=rospy.Publisher('/calc_traj_plots',Odometry,queue_size=10)
    rospy.loginfo("Mecanum_Waypoints has started !!")
    p_start = [0.0,0.0]
    i = 0
    t_prev = 0.0
    t_start = rospy.get_time()
    points = Vector3()
    plt_pt = Odometry ()
    t=rospy.get_time() - t_start
    rate=rospy.Rate(100)
    while not rospy.is_shutdown():
        
        # for Infinity
        #  A1 = 1.5 
        #  A2 = 1.5
        #  omega1 = 0.2094
        #  omega2 = 0.1047
        #  phi1 = -1.57
        #  phi2 = 0.0
        #  for Circle
         A1 = 1.0 
         A2 = 1.0
         omega1 = 0.2778
         omega2 = 0.2778
         phi1 = 0.0
         phi2 = 0.0
         T = 0.1     # low value for smooth velocity curve
         
         t_diff = t-t_prev
         if t_diff >= T and t <= 11.3  :
             x_t = A1 * np.cos(omega1*t + phi1) 
             y_t = A2 * np.sin(omega2*t + phi2) 
             x_t_dot = -1 * A1 * omega1 * np.sin(omega1*t+phi1) 
             y_t_dot =  A2 * omega2 * np.cos(omega2*t+phi2)

             points.x = float(x_t)                # updates values to send local goal points to traverse to mecanum_trajectory node
             points.y = float(y_t)                 
             plt_pt.pose.pose.position.x = float(x_t)  
             plt_pt.pose.pose.position.y = float(y_t)    
             plt_pt.twist.twist.linear.x = float(x_t_dot) 
             plt_pt.twist.twist.linear.y = float(y_t_dot)
             t_prev = t
             publish_plot_pts.publish(plt_pt)
             rospy.loginfo("values of x_t: " + str(x_t)+ " , y_t: " + str(y_t)+" ,Timestamp :" + str(t))
         publish_waypoints.publish(points)
         rospy.sleep(0.02)
         t = rospy.get_time() - t_start  
         rate.sleep()

def main(args=None):
    rospy.init_node('mecanum_waypoints' , anonymous=True)
    print("Node spinning will start")
    node_start()
    print("Node spinning will is ended") 
     
if __name__ == "__main__":
        main() 