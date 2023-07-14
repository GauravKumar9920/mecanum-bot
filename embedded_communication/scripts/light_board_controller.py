#!/usr/bin/env python
import rospy
import json
import time
import serial
import sys
from datetime import datetime
from std_msgs.msg import Float32
from embedded_communication.srv import LightControl, LightControlResponse
from embedded_communication.msg import Battery

from geometry_msgs.msg import Twist
from std_msgs.msg import String

root = {}
port_status = False
controller_alive = True
busy = False


json_data = rospy.get_param( '/embedded/light_config')
left_min = rospy.get_param( '/embedded/left_min')
left_max = rospy.get_param( '/embedded/left_max')
right_min = rospy.get_param( '/embedded/right_min')
right_max = rospy.get_param( '/embedded/right_max')
right_min2 = rospy.get_param( '/embedded/right_min2')
right_max2 = rospy.get_param( '/embedded/right_max2')

read_frequency= 10 
send_data = json.loads(json_data)
if send_data is None:
    rospy.logerr("parameter not load")

def check_data(rcv_data):
    try:
        data = json.loads(rcv_data)
        if data['response'] == True:
            if (root['board_no'] == data['board_no']) and (root['mode_no'] == data['mode_no']):
                for i in range(0, len(root['colour1'])):
                    if (root['colour1'][i] == data['colour1'][i]) and (root['colour2'][i] == data['colour2'][i]):
                        return True
                    else:
                        return False
            else:
                return False
        else:
            return False
    except:
        rospy.logerr("response data fail"+rcv_data)
        return False
    

def battery_voltage(value):
    global left_min,left_max,right_min2,right_max2
    left_span = left_max - left_min
    right_span = right_max2 - right_min2
    value_scaled = float(value - left_min) / float(left_span)
    return right_min2 + (value_scaled * right_span)

def get_sensor(event):
    try:
        if controller_alive and not busy:
            data = get('sensor')   
            sensor = Battery()
            rospy.loginfo("GET SENSOR DATA %s",data)
            try:
                if data:
                    data = json.loads(data)
                    if "battery" in data.keys():
                        battery = battery_voltage(data['battery'])
                        sensor.battery = battery
                    
                    if "charging" in data.keys():
                        charging = data["charging"]
                        sensor.charging = charging
                    
                    sensors.publish(sensor)
                    
                else:
                    rospy.logerr("No JSON GOT FROM MBED RESETTING SERIAL")
                    ser.close()
                    ser.open()

            except Exception as e:
                rospy.logerr("Get Sensor "+str(e))
            ser.reset_input_buffer()
        else:
            rospy.logerr("Get sensors not publishing Controller: %s, busy: %s",controller_alive,busy)

    except:
        rospy.logerr("read_write function error")
        return False


def write(data):
    try:
        data = json.dumps(data)+"\n"
        ser.write(data)
    except:
        rospy.logerr("Serial Write Failed")

def read():
    try:
        time.sleep(0.02)
        data = ser.readline()
    except:
        rospy.logerr("Serial Read Failed")
        data = ""
    return data

def rgb_control_cb(req):
    try:
        root['board_no'] = send_data['light'][req.light]
        root['mode_no'] = send_data['command'][req.command]
        root['colour1'] = req.clr1
        root['colour2'] = req.clr2
        root['port_no'] = send_data[req.light]['port_no']
        root['no_of_led'] = send_data[req.light]['count']
        root['r_speed'] =10
        root['f_back'] =10
        while busy:
            time.sleep(0.1)
        rcv_data = read_write(root)
        res = check_data(rcv_data)

        return LightControlResponse(response=res)
    except:
        rospy.logerr("rgb_control_cb function error")
        return LightControlResponse(response=False)


def read_write(data):
    global busy
    busy = True
    write(data)
    data = read()
    busy = False
    return data

def get(data):
    data = {"data_fetch": data}
    return read_write(data)


if __name__ == "__main__":
    rospy.init_node('io_control')
    rospy.loginfo("parameter load")
    rospy.Service('/light_control', LightControl, rgb_control_cb)
    sensors = rospy.Publisher("/battery", Battery, queue_size=1)

    try:
        ser = serial.Serial('/dev/LIGHT', 115200, timeout=0.3)
        rospy.loginfo("controller Port is available")
        port_status = True
        try:
            if port_status:
                alive = {"status": "Alive?"}
                alive_res = json.loads(read_write(alive))
                if alive_res['status'] == True:
                    controller_alive = True
                    rospy.loginfo("io_controller alive")
                    rospy.Timer(rospy.Duration(read_frequency), get_sensor)
                    controller_alive = True
                else:
                    controller_alive = False
                    rospy.logerr("io_controller not alive")
            else:
                rospy.logerr("control USB port not connected")
        except Exception as e:
            rospy.logerr("Service call failed" + str(e))
    except:
        rospy.logerr("controller Port is Unavailable")
        port_status = False
        time.sleep(2)
    rospy.spin()