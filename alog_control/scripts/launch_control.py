#!/usr/bin/env python
import roslaunch
import rospy
import time
from rospkg import RosPack
from std_msgs.msg import Bool
from embedded_communication.srv import LightControl
from queue import Queue

class LIGHT:
    def __init__(self):
        self.light_control = rospy.ServiceProxy('light_control',LightControl)

    class CONFIG:
        NAME = 'center_indication'
        MODE = 'b_off'
        EMG_MODE = 'b_on'
        IDLE_COLOR = [0,100,255]
        NAVIGATION_COLOR = [255,140,0]
        JOYSTICK_COLOR = [0,255,0]
        EMERGENCY_COLOR = [255,0,0]

    def set_navigation_color(self):
        res = self.light_control(LIGHT.CONFIG.NAME,LIGHT.CONFIG.MODE,LIGHT.CONFIG.NAVIGATION_COLOR,LIGHT.CONFIG.NAVIGATION_COLOR)
        rospy.loginfo("NAVIGATION COLOR SET RES: {}".format(res))

    def set_joystick_color(self):
        res = self.light_control(LIGHT.CONFIG.NAME,LIGHT.CONFIG.MODE,LIGHT.CONFIG.JOYSTICK_COLOR,LIGHT.CONFIG.JOYSTICK_COLOR)
        rospy.loginfo("JOYSTICK COLOR SET RES: {}".format(res))

    def set_idle_color(self):
        res = self.light_control(LIGHT.CONFIG.NAME,LIGHT.CONFIG.MODE,LIGHT.CONFIG.IDLE_COLOR,LIGHT.CONFIG.IDLE_COLOR)
        rospy.loginfo("IDLE COLOR SET RES: {}".format(res))
    
    def set_emergency_color(self):
        res = self.light_control(LIGHT.CONFIG.NAME,LIGHT.CONFIG.EMG_MODE,LIGHT.CONFIG.EMERGENCY_COLOR,LIGHT.CONFIG.EMERGENCY_COLOR)
        rospy.loginfo("EMERGENCY COLOR SET RES: {}".format(res))


class JoyControlRobot(LIGHT):
    def __init__(self):
        rospy.init_node("joy_control_robot")
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.ros_param_variables()
        self.dynamic_variables()
        self.ros_msgs()
        LIGHT.__init__(self)
        self.launch_queue = Queue()
        
        self.set_joystick_color()
        while not rospy.is_shutdown():
            time.sleep(0.1)
            if not self.launch_queue.empty():
                current_action = self.launch_queue.get()
                if current_action is 'start_navig' and not self.navigation_started and not self.emergency:
                    self.start_navigation()
                    rospy.wait_for_service("/move_base/get_loggers")
                    time.sleep(4)
                    self.set_navigation_color()
                elif current_action is 'stop_navig' and self.navigation_started and not self.emergency:
                    self.stop_navigation()
                    time.sleep(4)
                    self.set_joystick_color()
        
    def ros_param_variables(self):
        rospack = RosPack()
        self.navigation_package = rospy.get_param("/robot/navigation_package")
        self.navigation_launch_file = [rospack.get_path(self.navigation_package) + "/launch/navigation.launch"]
        
    def ros_msgs(self):
        rospy.Subscriber("/navigation_button_pressed",Bool,self.navig_pressed_cb,queue_size=1)
        rospy.Subscriber("/emergency",Bool,self.emergency_cb,queue_size=1)
    
    def dynamic_variables(self):
        self.navigation_launch = None
        self.navigation_started = False
        self.emergency = False

    def stop_navigation(self):
        if self.navigation_started:
            rospy.loginfo("Stopping Navigation")
            self.navigation_launch.shutdown()
            self.navigation_started = False 

    def start_navigation(self):
        self.navigation_launch = roslaunch.parent.ROSLaunchParent(self.uuid, self.navigation_launch_file)
        if not self.navigation_started:
            rospy.loginfo("Starting Navigation")
            self.navigation_launch.start()
            self.navigation_started = True

    def navig_pressed_cb(self,start):
        if start.data:
            rospy.loginfo("Navigation Start Request")
            self.launch_queue.put('start_navig')
        elif not start.data:
            rospy.loginfo("Navigation Stop Request")
            self.launch_queue.put('stop_navig')
    
    def emergency_cb(self,msg):
        if msg.data and msg.data != self.emergency:
            self.emergency = msg.data
            self.set_emergency_color()
            
        elif not msg.data and msg.data != self.emergency:
            self.emergency = msg.data
            if self.navigation_started:
                self.set_navigation_color()
            elif not self.navigation_started:
                self.set_joystick_color()

if __name__ == "__main__":
    JoyControlRobot()
        