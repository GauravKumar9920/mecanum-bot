#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math as np
import numpy as npy

class Data :
    def __init__(self):
        self.p_start = [0.0,0.0]
        self.points = Vector3()
        self.plt_pt = Odometry ()
        self.publish_waypoints = rospy.Publisher('/waypoints_to_follow',Vector3,queue_size=10)
        self.publish_plot_pts=rospy.Publisher('/calc_traj_plots',Odometry,queue_size=10)

def node_start():

    
    rospy.loginfo("Mecanum_Waypoints has started !!")
    global data
    data = Data()
    i = 0
    t_prev = 0.0
    t_start = rospy.get_time()
    t=rospy.get_time() - t_start
    # waypoints=[[2.5 ,0.0,8], [2.5, -2.5,3.0], [5, -2.5,1.5], [5, 0,1.5], [5, 2.5,1.5],
                    #   [7.5, 2.5,1.5], [7.5, 0,1.5], [7.5, -2.5,1.5], [5.0, -2.5,1.5], [5.0, 0,1.5],
                    #   [5.0, 2.5,1.5], [2.5, 2.5,1.5], [2.5, 0,1.5], [0, 0,1.5]]
    waypoints=[[2.4585770797112769,1.1843450794605883,8.5], [-0.08034108032076903,2.1059354489412897,8.5], [-2.367353160458823504,1.6052357912808781,8.0], [-1.796658166081795,-1.2535191362468456,9], [0.8383782330688945,-2.19211086808546,8.5],
                      [2.095903349334137925,-2.172783671538676,3.2], [2.4986470181651813,0.18784494633686227,7.5]] #[7.5, -2.5,4.5], [5.0, -2.5,2.0], [5.0, 0,1.5],
                    #   [5.0, 2.5,1.5], [2.5, 2.5,1.5], [2.5, 0,1.5], [0, 0,1.5]]
         
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
         
         T = waypoints[i][2]
       
         t = rospy.get_time() - t_start
         t_diff = t-t_prev
         if t_diff >= T :
                   move_to_point(waypoints[i])                 
                   t_prev = t                
                             
                   if i<=5 :
                        i += 1
                   else :
                        i = 0
                   rospy.loginfo("set Points set !!" + str(i) +" x:" + str(waypoints[i][0]) + ", y:" + str(waypoints[i][1])+" t_diff :" + str(t_diff)) 
         
         rate.sleep()

def move_to_point(current_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        
        # self.get_logger().info(str(p_start))
        start_time=rospy.get_time()
        p_end=current_waypoint[0:2]
        T=current_waypoint[2]
        v_start=[0,0]
        v_end=[0,0]
        A,B=polynomial_time_scaling_3rd_order(data.p_start,v_start,p_end,v_end,T)
        t = rospy.get_time()-start_time
        prev_time = 0
        
        while(t<=T) :
        
            diff_time = t - prev_time 
            if(diff_time >= 0.1) :           # 0.25 is sample time here,
                x_t = A[0] +( A[1] * t ) + (A[2] * t**2) + (A[3] * t**3)
                y_t = B[0] +( B[1] * t ) + (B[2] * t**2) + (B[3] * t**3)
                x_t_dot = A[1] + 2*A[2]*t + 3 * A[3] * (t**2)
                y_t_dot = B[1] + 2*B[2]*t + 3 * B[3] * (t**2)
                data.points.x = float(x_t)                # updates values to send local goal points to traverse to mecanum_trajectory node
                data.points.y = float(y_t)                 
                data.plt_pt.pose.pose.position.x = float(x_t)  
                data.plt_pt.pose.pose.position.y = float(y_t)    
                data.plt_pt.twist.twist.linear.x = float(x_t_dot) 
                data.plt_pt.twist.twist.linear.y = float(y_t_dot)
                data.publish_plot_pts.publish(data.plt_pt)                    
                prev_time = t
                # self.get_logger().info("go_to_goal is called x_t:" + str(x_t) + "y_t:" + str(y_t)+" ,timestamped :"+str(t))
            t = rospy.get_time()-start_time
            
            data.publish_waypoints.publish(data.points)
            rospy.sleep(0.02)
        data.p_start = current_waypoint[0:2]
        rospy.loginfo("Move_to_point is completed" )    
        

def polynomial_time_scaling_3rd_order( p_start, v_start, p_end, v_end, T):
    # input: p,v: position and velocity of start/end point
    #        T: the desired time to complete this segment of trajectory (in second)
    # output: the coefficients of this polynomial

    xi=p_start[0]
    yi=p_start[1]
    xf=p_end[0]
    yf=p_end[1]

    xi_dot=v_start[0]
    yi_dot=v_start[1]
    xf_dot=v_end[0]
    yf_dot=v_end[1]
    
    X = npy.array([[xi], 
                    [xi_dot],
                    [xf],
                    [xf_dot]])
    
    Y = npy.array([[yi], 
                    [yi_dot],
                    [yf], 
                    [yf_dot]])
    
    T = npy.array([[1,0,0,0 ], 
                    [0,1,0,0 ],
                    [1,T,T**2,T**3 ], 
                    [0,T,2*T,3*(T**2) ]])
    
    inv_T = npy.linalg.inv(T)
    A = npy.dot(inv_T,X)
    B = npy.dot(inv_T,Y) 

    return A,B     
        
def main(args=None):
    rospy.init_node('mecanum_waypoints' , anonymous=True)
    print("Node spinning will start")
    node_start()
    print("Node spinning will is ended") 
     
if __name__ == "__main__":
        main() 