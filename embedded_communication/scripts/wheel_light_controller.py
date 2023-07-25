#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from embedded_communication.srv import LightControl,LightControlRequest
import numpy
import json
import sys

controller_alive = False
reverse = False
last_state = None
busy = False


left_light = rospy.get_param('/wheel_light/left', None)
right_light = rospy.get_param('/wheel_light/right', None)
forward_light = rospy.get_param('/wheel_light/forward',None)
reverse_light = rospy.get_param('/wheel_light/reverse',None)
brake_light = rospy.get_param('/wheel_light/brake',None)

Left = json.loads(left_light)
Right = json.loads(right_light)
Forward = json.loads(forward_light)
reverse = json.loads(reverse_light)
brake = json.loads(brake_light)

def rgb_cb(light, command, colour1, colour2):
    rgb = rospy.ServiceProxy('/light_control', LightControl)
    try:
        resp1 = rgb(light, command, colour1, colour2)
        return resp1.response
    except rospy.ServiceException as e: 
        rospy.logerr("Service call failed: %s"%e)
        return False

def stop():
    try:
        for i in range (0,len(brake["brake_light"])):
            ligt =  brake["brake_light"][i]
            cmd = brake["brake_mode"][i]
            clr1 = brake["brake_clr1"][i]
            clr2 = brake["brake_clr2"][i]
            res = rgb_cb(ligt, cmd, clr1, clr2)
            if res == False :
                res = rgb_cb(ligt, cmd, clr1, clr2)
    except:
        rospy.logerr("stop function error")

def rev():
    try:
        for i in range (0,len(reverse["rev_light"])):        
            ligt =  reverse["rev_light"][i]
            cmd = reverse["rev_mode"][i]
            clr1 = reverse["rev_clr1"][i]
            clr2 = reverse["rev_clr1"][i]
            res = rgb_cb(ligt, cmd, clr1, clr2)
            if res == False :
                res = rgb_cb(ligt, cmd, clr1, clr2)
    except:
        rospy.logerr("rev function error")

def forward():
    try:
        for i in range (0,len(Forward["forward_light"])):
            ligt =  Forward["forward_light"][i]
            cmd = Forward["forward_mode"][i]
            clr1 = Forward["forward_clr1"][i]
            clr2 = Forward["forward_clr2"][i]
            res = rgb_cb(ligt, cmd, clr1, clr2)
            if res == False :
                res = rgb_cb(ligt, cmd, clr1, clr2)
    except:
        rospy.logerr("forward function error")

def right():
    try:
        for i in range (0,len(Right["right_light"])):
            ligt =  Right["right_light"][i]
            cmd = Right["right_mode"][i]
            clr1 = Right["right_clr1"][i]
            clr2 = Right["right_clr2"][i]
            res = rgb_cb(ligt, cmd, clr1, clr2)
            if res == False :
                res = rgb_cb(ligt, cmd, clr1, clr2)
    except:
        rospy.logerr("right function error")

def left():
    try:
        for i in range (0,len(Left["left_light"])):
            ligt =  Left["left_light"][i]
            cmd = Left["left_mode"][i]
            clr1 = Left["left_clr1"][i]
            clr2 = Left["left_clr2"][i] 
            res = rgb_cb(ligt, cmd, clr1, clr2)
            if res == False :
                res = rgb_cb(ligt, cmd, clr1, clr2)
    except:
        rospy.logerr("left function error")


def cmd_vel_cb(data):
    global last_state,busy
    lin = data.linear.x
    yaw = data.angular.z
    omni = data.linear.y
    while busy:
        pass
    if lin == 0.0 and yaw == 0.0 and omni == 0.0:
        if last_state != "idel":
            busy = True
            stop()
        last_state= "idel"
        busy = False
    if numpy.sign(lin) == -1 and omni < 0.2 and yaw < 0.3:
        if last_state != "backward":
            busy = True
            rev()
        last_state = "backward"
        busy = False
    elif numpy.sign(lin) == 1 and omni < 0.2 and yaw < 0.3:
        if last_state != "forward":
            busy = True
            forward()
        last_state = "forward"
        busy = False
    
    if abs(omni) > 0.2:
        if numpy.sign(omni) == -1 and lin == 0.0:
            if last_state != "right":
                busy = True
                right()
            last_state = "right"
            busy = False
        elif numpy.sign(omni) == 1 and lin == 0.0:
            if last_state != "left" :
                busy = True
                left()
            last_state = "left"
            busy = False
        
        elif (numpy.sign(omni) == 1 or numpy.sign(omni) == -1) and numpy.sign(lin) == -1:
            if last_state != "backward":
                busy = True
                rev()
            last_state = "backward"
            busy = False
        
        elif numpy.sign(omni) == 1 and numpy.sign(lin) == 1:
            if last_state != "left" :
                busy = True
                left()
            last_state = "left"
            busy = False
        
        elif numpy.sign(omni) == -1 and numpy.sign(lin) == 1:
            if last_state != "right":
                busy = True
                right()
            last_state = "right"
            busy = False

            
    if abs(yaw) > 0.3:
        if numpy.sign(yaw) == -1 and lin == 0.0:
            if last_state != "right":
                busy = True
                right()
            last_state = "right"
            busy = False
        
        elif numpy.sign(yaw) == 1 and lin == 0.0:
            if last_state != "left" :
                busy = True
                left()
            last_state = "left"
            busy = False
        
        elif numpy.sign(yaw) == -1 and numpy.sign(lin) == 1:
            if last_state != "right":
                busy = True
                right()
            last_state = "right"
            busy = False
        
        elif numpy.sign(yaw) == 1 and numpy.sign(lin) == 1:
            if last_state != "left" :
                busy = True
                left()
            last_state = "left"
            busy = False
        
        elif numpy.sign(yaw) == -1 and numpy.sign(lin) == -1:
            if last_state != "left" :
                busy = True
                left()
            last_state = "left"
            busy = False
        
        elif numpy.sign(yaw) == 1 and numpy.sign(lin) == -1:
            if last_state != "right":
                busy = True
                right()
            last_state = "right"
            busy = False

    


def main():
    try:
        rospy.Subscriber('/cmd_vel',Twist,cmd_vel_cb)
        rospy.spin()
    except KeyboardInterrupt:
        sys.exit(0)
    except:
        rospy.logerr("Subscriber error")

if __name__ == "__main__":
    try:
        rospy.init_node('rgb_speed_controler',log_level=rospy.INFO)
        time.sleep(3)
        stop()
        main()
    except:
        rospy.logerr("node initiate error")
