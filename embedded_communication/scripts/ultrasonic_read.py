#!/usr/bin/env python
import rospy
import serial
import json
import time
from os import system
from std_msgs.msg import Bool
from embedded_communication.msg import UltraSonic,Buttons,Cliff
from embedded_communication.srv import HardEmergency,HardEmergencyRequest,HardEmergencyResponse
from embedded_communication.srv import Siren,SirenResponse

port = '/dev/ULTRA'
port_status = False

def read():
    try:
        time.sleep(0.005)
        if port_status:
            while not rospy.is_shutdown():
                data = ser.readline()
                publish_data(data)
    except:
        rospy.logerr("read error")

def publish_data(data):
    if port_status :
        try:
            data = json.loads(data)
            data_keys = data.keys()
            ultra = UltraSonic()
            clif = Cliff()
            button = Buttons()

            if "cliff" in data_keys:
                sensor1 = (float(data["cliff"][0][3:6]))/10
                sensor2 = (float(data["cliff"][1][3:6]))/10
                cliffs = [sensor1,sensor2]
                clif.cliff = cliffs
                cliff_pub.publish(clif)

            if "bumper" in data_keys:
                bumper = data["bumper"]
                bumper_pub.publish(bumper)                                                     

            if 'u' in data_keys:
                distances = data['u']
                for i in range(0,len(distances)):
                    distances[i] = float(distances[i])
                    if distances[i] == None or distances[i] <= 4.0:
                        distances[i] = 0.0
                    
                ultra.distances = distances
                ultra_pub.publish(ultra)

            if 'emg' in data_keys:
                button = data['emg']
                button_pub.publish(button)
               
        except:
            rospy.logerr("publish data function error")
            pass

if __name__ == "__main__":
    rospy.init_node("ultrasonic_node")
    try:
        ser = serial.Serial(port= port,baudrate=115200, timeout=1)
        port_status = True
        rospy.loginfo("UltraSonic Port is Available")
    except:
        port_status = False
        rospy.logerr("UltraSonic Port is Unavailable")
        time.sleep(2)
    
    ultra_pub = rospy.Publisher("/ultrasonic_data",UltraSonic,queue_size=1)
    cliff_pub = rospy.Publisher("/cliff_data", Cliff,queue_size = 1)
    bumper_pub = rospy.Publisher("/bumper_data", Bool, queue_size = 1)
    button_pub = rospy.Publisher("/emergency",Bool,queue_size=1)
    read()
    rospy.spin()