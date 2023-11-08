#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
x = z = 0.0
l_speed = 0.3
a_speed = 0.5


def joyCB(msg):
    global l_speed, a_speed
    if msg.buttons[5]:
        l_speed = l_speed + (l_speed/20)
        a_speed = a_speed + (a_speed/20)
        print ("lin:", "{:.3f}".format(l_speed), "ang", "{:.3f}".format(a_speed))

    if msg.buttons[7]:
        l_speed = abs(l_speed - (l_speed/20))
        a_speed = abs(a_speed - (a_speed/20))
        print ("lin:", "{:.3f}".format(l_speed), "ang", "{:.3f}".format(a_speed))
    if msg.buttons[6] == 1:
        if not msg.axes[2]:
            if msg.axes[5] == 1:
                x = l_speed
                z = 0
                speed_pub(x, z)
            elif msg.axes[5] == -1:
                x = - l_speed
                z = 0
                speed_pub(x, z)
            elif msg.axes[4] == -1:
                x = 0
                z = - a_speed
                speed_pub(x, z)
            elif msg.axes[4] == 1:
                x = 0
                z = a_speed
                speed_pub(x, z)
            ## holonomic integ starts
            elif msg.buttons[2] == 1: ## go right linear
                x = 0
                y = -l_speed
                z = 0
                speed_pub(x, z,y)
            elif msg.buttons[0] == 1: ## go left linear
                x = 0
                y = l_speed
                z = 0
                speed_pub(x, z,y)
            elif msg.axes[1] == 1.0: ## go left diagonal forward
                x = l_speed
                y = l_speed
                z = 0
                speed_pub(x, z,y)
            elif msg.axes[1] == -1.0: ## go left diagonal reverse
                x = -l_speed
                y = l_speed
                z = 0
                speed_pub(x, z,y)
            elif msg.axes[3] == 1.0: ## go right diagonal forward
                x = l_speed
                y = -l_speed
                z = 0
                speed_pub(x, z,y)
            elif msg.axes[3] == -1.0: ## go right diagonal reverse
                x = -l_speed
                y = -l_speed
                z = 0
                speed_pub(x, z,y)
            ### holonomic integ ends
            elif msg.buttons[3] == 1.0:
                navig_pub.publish(True)
            elif msg.buttons[1] == 1.0:
                navig_pub.publish(False)
            else:
                x = 0
                z = 0
                speed_pub(x, z)
        if msg.axes[5] and int(msg.axes[2]):
            if msg.axes[5] == 1 and int(msg.axes[2]) == -1:
                x = l_speed
                z = -a_speed
                speed_pub(x, z)
            elif msg.axes[5] == 1 and int(msg.axes[2]) == 1:
                x = l_speed
                z = a_speed
                speed_pub(x, z)
            elif msg.axes[5] == -1 and int(msg.axes[2]) == 1:
                x = -l_speed
                z = a_speed
                speed_pub(x, z)
            elif msg.axes[5] == -1 and int(msg.axes[2]) == -1:
                x = -l_speed
                z = -a_speed
                speed_pub(x, z)

    else:
        x = 0
        z = 0
        speed_pub(x, z)


def speed_pub(x, z,y=0.0):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.angular.z = z
    vPub.publish(twist)


if __name__ == '__main__':
    rospy.init_node("holonomic_joystick_control_node")
    rospy.Subscriber("/joy", Joy, joyCB)
    vPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    navig_pub = rospy.Publisher("/navigation_button_pressed",Bool,queue_size=1)
    rospy.spin()
