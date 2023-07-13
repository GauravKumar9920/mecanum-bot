#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist,Vector3
from nav_msgs.msg import Odometry 
import math as np


class Data :
    def __init__(self):
        self.i=0
        # self.path=[(0,0),(5.0,0.0),(2.0,-3.0),(-4.0,-4.0),(-6.0,-9.0),(-2.0,-0.5)]
        self.path=[(2.5,-0.5,1.78),(-1.5,0.9,0)]
                #    (4.9608,0.6246),(4.9120,0.9339),( 4.8439,1.2395),( 4.7569,1.5402),(4.6511,1.8349),(4.5272,2.1224),(4.3855,2.4016),(4.2266,2.6713),(4.0511,2.9306 ),(3.8598,3.1784),(3.6533,3.4137),(3.4325,3.6357),(3.1982,3.8434),(2.9514,4.0360),(2.6930,4.2128),(2.4241,4.3731),(2.1456,4.5162),(1.8588,4.6417)]
        # self.path=[(4.85770797112769,1.1843450794605883),
        # (4.219781652239277,2.682059433984407),
        # (3.5902321201188965,3.479976052168545),
        # (2.796054321856252,4.145127287458007),
        # (1.8933492941067267,4.627659068093236),
        # (0.9372938224685189,4.911362366020486),
        # (-0.08034108032076903,4.999354489412897),
        # (-1.0767191811366583,4.88269145092872),
        # (-2.0522775860613587,4.559403108933248),
        # (-2.9205948119540466,4.058340294305926),
        # (-3.6763972798046636,3.38882030226521),
        # (-4.28559421082663,2.575593574329862),
        # (-4.7353160458823504,1.6052357912808781),
        # (-4.964746139335526,0.5927020937646448),
        # (-4.980768045912425,-0.43812061446332823),
        # (-4.784126887941143,-1.4533168684352997),
        # (-4.377079823233171,-2.4168517168095103),
        # (-3.796658166081795,-3.2535191362468456),
        # (-3.0492404624485814,-3.96259165221025),
        # (-2.1998305192739736,-4.490071902149318),
        # (-1.2291397143361453,-4.84656739998957),
        # (-0.18964016972233727,-4.996402366305949),
        # (0.8383782330688945,-4.929211086808546),
        # (1.8266083036461358,-4.654406740396781),
        # (2.742274501136914,-4.180900687700474),
        # (3.5429333500820404,-3.528118943130525),
        # (4.1903349334137925,-2.72783671538676),
        # (4.650749838265213,-1.8358992188777925),
        # (4.926613631332883,-0.8535093013933834),
        # (4.996470181651813,0.18784494633686227)]
        
        
        self.kp_w= 1.48 ## values so far worked were 0.95,0.58,0.0009 
        self.kd_w=0.460  #0.58
        self.ki_w=0.0001   #0.0009
        self.e_orient_int=0
        self.e_orient_old=0

        self.kp_v_x=1.79
        self.kd_v_x=0.68
        self.ki_v_x=0.00018

        self.kp_v_y=1.68
        self.kd_v_y=0.59
        self.ki_v_y=0.00018

        self.e_x_int=0
        self.e_x_old=0

        self.e_y_int=0
        self.e_y_old=0

        self.prev_time=0
        self.publish_=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.publish_euler = rospy.Publisher("/quat_to_euler",Vector3,queue_size=10)
def node_start():
        
        global data 
        data = Data()
        subscriber_=rospy.Subscriber("/odom",Odometry,odometry_data_callback)
        rospy.loginfo("Mecanum_go_to_goal_controller has started !!")
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
    
    data.curr_yaw = quat_to_euler(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w)
    # rospy.loginfo("odom data x: "+str(data.x)+" y: "+str(data.y)+" yaw: "+str(data.curr_yaw))
    euler = Vector3()
    euler.z = 57.296*data.curr_yaw #in degrees
    data.publish_euler.publish(euler)
    data.des_x ,data.des_y ,data.des_orient = data.path[data.i]

    delta_angle =data.curr_yaw
#     angle_diff=np.atan2((data.des_y-odom.pose.pose.position.y),(data.des_x-odom.pose.pose.position.x))
    angle_diff=np.atan2((data.des_y),(data.des_x))
    e_orient=data.des_orient - delta_angle
#     rospy.loginfo("Angle_diff in radians : " + str(e_orient))
    
   
    #  e_x=np.sqrt(pow((data.des_x-odom.pose.pose.position.x),2)+pow((data.des_y-odom.pose.pose.position.y),2))
    #  rospy.loginfo("dist_diff in units : " + str(e_x))

    e_x=data.des_x-odom.pose.pose.position.x
    e_y=data.des_y-odom.pose.pose.position.y
    rospy.loginfo("errors  x: " + str(e_x) +" ,y: " + str(e_y))

    
    
            
    data.e_x_int+=e_x
    data.e_y_int+=e_y
    data.e_orient_int+=e_orient
    
    data.e_x_old=e_x
    data.e_y_old=e_y
    data.e_orient_old=e_orient

    e_orient_diff=e_orient-data.e_orient_old
    e_x_diff=e_x-data.e_x_old
    e_y_diff=e_y-data.e_y_old

    u_w=data.kp_w*e_orient+data.kd_w*e_orient_diff+data.ki_w*data.e_orient_int
    u_x=data.kp_v_x*e_x+data.kd_v_x*e_x_diff+data.ki_v_x*data.e_x_int
    u_y=data.kp_v_y*e_y+data.kd_v_y*e_y_diff+data.ki_v_y*data.e_y_int 
    

    if u_x >= 0.5 :
            u_x = 0.5
    elif u_x <= -0.5 :
            u_x = -0.5

    if u_y >= 0.5 :
            u_y = 0.5
    elif u_y <= -0.5 :
            u_y = -0.5
    
    if u_w >= 0.8 :
            u_w = 0.8
    elif u_w <= -0.8 :
            u_w = -0.8
            
#     if np.pow(e_x**2 + e_y**2,0.5) <= 0.01 :
    if e_orient <= 0.009 and e_orient >= -0.009:
        # if (data.i <=1):
        #     data.i +=1
            u_w = 0
            u_x = 0
            u_y = 0
            
        #Frame transformations from Global to Body frame
    v_x = (u_x * np.cos(data.curr_yaw)) + (u_y * np.sin(data.curr_yaw))
    v_y = - (u_x * np.sin(data.curr_yaw)) + (u_y * np.cos(data.curr_yaw))  
    v_w = u_w
        
    cmd=Twist()
#     cmd.linear.x= float( v_x)
#     cmd.linear.y= float(v_y)
    cmd.linear.z=0.0
    cmd.angular.x=0.0
    cmd.angular.y=0.0
    cmd.angular.z= float(v_w)
    # data.publish_.publish(cmd)

def main(args=None):
    rospy.init_node('go_to_goal', anonymous=True)
    print("Node spinning will start")
    node_start()
    print("Node spinning will is ended") 
     
if __name__ == "__main__":
        main()
