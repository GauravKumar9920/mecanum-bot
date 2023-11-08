#!/usr/bin/env python
'''
This program is used to control Leadshine Servo driver , Get encoder data and Alarm status and publishes it.

'''
# Subscribed Topics: /cmd_vel, /buttons_state
# Published Topics: /motor_$(motor_name)/encoder,/motor_$(motor_name)/alarm
# service Topics: /motor_$(motor_name)/speed ,/motor_$(motor_name)/voltage

import swri_rospy
import rospy
import json
import math
import time
import serial
import numpy as np
import sys
import atexit
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Int32, String, Bool
from std_srvs.srv import Trigger, TriggerResponse
from motor_driver.srv import Speed, SpeedResponse, Voltage, VoltageResponse, WriteRegister, WriteRegisterResponse
from motor_driver.srv import ReadRegister,ReadRegisterResponse
from embedded_communication.msg import Buttons
import os
from std_msgs.msg import Float32MultiArray

connected = False
com_delay = 0.01
last_x = 0
last_z = 0
alarm_status = False

linear_limit = 1.5
angular_limit = 2.0

# encoder
prev_enc = 0
enc_corrected = 0

motor_name = sys.argv[1]
print("************* motor_name ***************", motor_name)

acceleration = rospy.get_param('/motor/acceleration')
deceleration = rospy.get_param('/motor/deceleration')

salve_id = int(rospy.get_param('/motor/slave_id'))

wheel_radius = rospy.get_param('/motor/wheel_radius')

robot_width = float(rospy.get_param('/motor/robot_width'))

gear_ratio = rospy.get_param('/motor/gear_ratio')

max_speed_p = rospy.get_param('/motor/max_speed_p')

max_speed_n = rospy.get_param('/motor/max_speed_n')

baudrate = int(rospy.get_param('/motor/baudrate'))

inverted = bool(rospy.get_param('/motor/'+motor_name+"/inverted"))

motor_index = int(rospy.get_param('/motor/'+motor_name+"/index"))

invert_encoder = bool(rospy.get_param('/motor/invert_encoder'))

port = rospy.get_param('/motor/'+motor_name + '/port')

serial_low_latency = (rospy.get_param('/motor/serial_low_latency'))

if serial_low_latency:
    os.system ("setserial "+port+" low_latency")

motor_id = chr(salve_id)

m_read = '\x03'
s_write = '\x06'
m_write = '\x10'
run = False

add_spd = '\x62\x00'
add_pos = '\x60\x2C'
add_alrm = '\x01\xF2'
add_alrm_clr = '\x01\x9A'
add_e_stop = '\x60\x02'

print("***** port *****", port)
rospy.loginfo("INFO:   Acceleration: %s", acceleration)
rospy.loginfo("INFO:   Deceleration: %s", deceleration)

##__convert integer to HEX__##


def tohex(val, shift):
    return hex((val + (1 << shift)) % (1 << shift))

##__calculate crc value__##


def crc16(data):
    '''
    CRC-16-ModBus Algorithm
    '''
    data = bytearray(data)
    poly = 0xA001
    crc = 0xFFFF
    for b in data:
        crc ^= (0xFF & b)
        for _ in range(0, 8):
            if (crc & 0x0001):
                crc = ((crc >> 1) & 0xFFFF) ^ poly
            else:
                crc = ((crc >> 1) & 0xFFFF)
    crc = tohex(crc, 16)
    byte = '0xff'
    d1 = int(tohex((int(crc, 16) & int(byte, 16)), 16), 16)
    d2 = int(tohex((int(crc, 16) >> 8), 16), 16)
    data = chr(d1) + chr(d2)
    return data

##__check if the crc of the response is correct__##


def crc_check(data):
    calc_crc = crc16(data[:-2])
    crc = data[-2:]
    crc = crc[0] + crc[1]
    if calc_crc == crc:
        return True
    else:
        rospy.logerr("ERROR:  " + motor_name + " Motor: CRC Check Failed!")
        return False

##__check if the speed write is done sussessfully__##


def write_chk(motor, val):
    if val == chr(motor) + '\x10\x62\x00\x00\x08\xde\x77':
        return True
    else:
        rospy.loginfo("ERROR:  " + motor_name + " Motor: Write Failed!")
        return False


##__convert input integer data to a 16 BIT ASCII HEX format__##
def todata(val):
    val = tohex(val, 16)
    byte = '0xff'
    d2 = int(tohex((int(val, 16) & int(byte, 16)), 16), 16)
    d1 = int(tohex((int(val, 16) >> 8), 16), 16)
    data = chr(d1) + chr(d2)
    return data

##__calculate the final HEX data of speed to be written to the driver__##


def calc(motor, speed):
    data = motor + m_write + add_spd + '\x00\x08\x10\x00\x02\x00\x00\x00\x00' + \
        speed + acc + dcc + '\x00\x00\x00\x10'
    crc = crc16(data)
    data = data + crc
    return data

##__writes the speed data to the driver and reads the accknowledgement__##


def motor_control(motor, val):
    global acc, dcc
    speed = todata(val)
    acc = todata(acceleration)
    dcc = todata(deceleration)
    data = calc(motor, speed)
    ser.write(bytes(data))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            return write_chk(motor, out)
        else:
            return False
    except:
        pass

##__reads the encoder value__##


def pos_read(motor):
    get_pos = motor + m_read + add_pos + '\x00\x02'
    crc = crc16(get_pos)
    get_pos = get_pos + crc
    ser.write(bytes(get_pos))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            ticks = ord(out[3])*16**6 + ord(out[4])*16**4 + \
                ord(out[5])*16**2 + ord(out[6])
            position = -(ticks & 0x80000000) | (ticks & 0x7fffffff)
            return position
    except:
        pass

##__reads the alarm__##


def alarm_read(motor):
    global alarm
    get_alrm = motor + m_read + add_alrm + '\x00\x01'
    crc = crc16(get_alrm)
    get_alrm = get_alrm + crc
    ser.write(bytes(get_alrm))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            data1 = tohex(ord(out[3]), 16)
            data2 = tohex(ord(out[4]), 16)
            data = data1[2:] + data2[2:]
            data = data[-3:]
            alarm = alarm_chk(out[0], data)
            if alarm:
                alarm_stop(True)
                alarm = alarm_clear(motor)
    except:
        pass

##__clears the alarm__##


def alarm_clear(motor):
    global alarm_status
    rospy.loginfo("INFO:   clearing Alarm in " + motor_name + " Motor")
    alrm_clr = motor + s_write + add_alrm_clr + '\x77\x77'
    crc = crc16(alrm_clr)
    alrm_clr = alrm_clr + crc
    ser.write(bytes(alrm_clr))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            if out == alrm_clr:
                rospy.loginfo("INFO:   Alarm Cleared in " +
                              motor_name + " Motor")
                alarm_status = False
                return alarm_status
        return False
    except:
        return False

##__check the type of alarm__##


def alarm_chk(motor, data):
    global alarm_status
    if data == '00':
        alarm_status = False
    elif data == '0e1' or data == '0e0':
        alarm_status = True
        message = "ERROR: Over Current!"
    elif data == '100':
        alarm_status = True
        message = "ERROR: Over Load!"
    elif data == '180':
        alarm_status = True
        message = "ERROR: Position Error Over-Large!"
    elif data == '0a0':
        alarm_status = True
        message = "ERROR: Over Speed!"
    elif data == '0a1':
        alarm_status = True
        message = "ERROR: Speed Out of Control!"
    elif data == '0d0':
        alarm_status = True
        message = "ERROR: DC Bus Under Voltage!"
    elif data == '0c0':
        alarm_status = True
        message = "ERROR: DC Bus Over Voltage!"
    elif data == '171' or data == '172':
        alarm_status = True
        message = "ERROR: Encoder Parameters Read Error!"
    elif data == '190':
        message = "ERROR: Excessive Vibration!"
        alarm_status = True
    elif data == '150':
        message = "ERROR: Encoder Line Breaked!"
        alarm_status = True
    elif data == '151' or data == '170':
        message = "ERROR: Encoder Data Error!"
        alarm_status = True
    elif data == '152':
        message = "ERROR: Initilized Position of Encoder Error!"
        alarm_status = True
    elif data == '240':
        message = "ERROR: CRC Verification Error When EEPROM Parameter is Saved!"
        alarm_status = True
    elif data == '570':
        message = "ERROR: This Error has no Documentation!"
        alarm_status = True
    elif data == '120':
        message = "ERROR: Resistance Dischage Circuit Over-Load!"
        alarm_status = True
    elif data == '153':
        message = "ERROR: Encoder Battery Error!"
        alarm_status = True
    elif data == '210' or data == '211' or data == '212':
        message = "ERROR: I/F Input Interface Allocation Error!"
        alarm_status = True
    else:
        alarm_status = True
        message = "ERROR: Unknown Error " + data + \
            " in " + motor_name + " Motor: Unknown Error!!!"
    if alarm_status:
        alarm_pub.publish(message)
        rospy.logwarn(message)
    return alarm_status

##__write which data to be stored on which register to read the status__##


def status_write(motor):
    global motor_status
    status = motor + m_write + '\x01\x91\x00\x02\x04\x01\x40\x00\x41'
    crc = crc16(status)
    status = status + crc
    ser.write(bytes(status))
    print("wrote", status)
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        print("out", out)
        if crc_check(out):
            motor_status = True
        else:
            motor_status = False
        return motor_status
    except:
        return False

##__reads the statuses DC Bus Voltage and speed of motor__##


def status_read(motor, no):
    ser.flush()
    status = motor + m_read + '\x01\xF3\x00\x02'
    crc = crc16(status)
    status = status + crc
    ser.write(bytes(status))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        voltage = (ord(out[3])*16**2 + ord(out[4]))/10.0
        speed = ord(out[5])*16**2 + ord(out[6])
        if speed >= 32767:
            speed = speed - 65535
        if no == 1:
            return voltage
        elif no == 2:
            return speed
    except:
        pass


def alarm_stop(val):
    rospy.loginfo("WARNING:Alarm Emergency Stop")
    e_stop = motor_id + s_write + add_e_stop + '\x00\x40'
    crc = crc16(e_stop)
    e_stop = e_stop + crc
    ser.write(bytes(e_stop))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        rospy.loginfo("INFO:   " + motor_name +
                      " Motor: Alarm Emergency Stop Success")
    except:
        rospy.loginfo("ERROR:  " + motor_name +
                      " Unable to Alarm Emergency Stop! Try Again!")


def speed_lim(data):
    global max_speed_p, max_speed_n
    if data >= max_speed_p:
        speed = max_speed_p
    elif data <= max_speed_n:
        speed = max_speed_n
    else:
        speed = data
    return speed


def cb(msg):
    global gear_ratio
    distributed_speed = json.loads(msg.data)
    if run and not alarm_status:
        rpm = distributed_speed[motor_name]
        if inverted:
            rpm = -1 * rpm
        rpm = rpm * 9.54929659642538 ## conversion of rad/s to rpm
        speed = int(rpm * gear_ratio)
        speed = speed_lim(speed)
        motor = motor_control(motor_id, speed)
    else:
        rospy.loginfo("The motors are not ready to run")

##__reads and publishes encoder data__##


def enc_cb(event):
    global prev_enc, enc_corrected
    if run:
        enc = pos_read(motor_id)
        if enc != None:
            if invert_encoder:
                enc = -1 * enc
            if enc == 0:
                prev_enc = enc_corrected
            enc_corrected = prev_enc + enc
            enc_pub.publish(Int64(enc_corrected))

##__reads alarm data__##


def alarm_cb(event):
    if run:
        alarm_read(motor_id)

##__reads and publishes motor speed__##


def status_cb(request):
    rospy.loginfo('speed service called')
    speed = status_read(motor_id, 2)
    return SpeedResponse(speed=speed)

##__reads the voltage__##


def voltage_cb(request):
    rospy.loginfo('voltage service called')
    voltage = status_read(motor_id, 1)
    return VoltageResponse(voltage=voltage)

##__check if the motors are connected or not__##


def motor_stat(motor):
    if status_write(motor):
        rospy.loginfo("INFO:   " + motor_name + " Motor: Connected")
        return True
    else:
        rospy.loginfo("WARNING:" + motor_name + " Motor: Not Connected")
        return False


def motor_check():
    global run
    print("motor_check called")
    if motor_stat(motor_id):
        run = True
        rospy.loginfo("INFO:   Ready to run")
        time.sleep(0.3)
    else:
        run = False
        rospy.loginfo("WARNING:Not Ready to run")


def exit_stop():
    motor_control(motor_id, 0)
    rospy.loginfo("WARNING:Servo Driver Node closed! Stopped " +
                  motor_name + " motor!")


def buttons_cb(msg):
    global run
    if msg.data and run:
        run = False
        rospy.logwarn("HARWARE EMERGENCY PRESSED")
    elif not msg.data and not run:
        rospy.logwarn("HARWARE EMERGENCY RELEASED")
        ser.flushInput()
        run = True
        time.sleep(3)
        motor_check()

# to adjust stiffness parameter of motor drive


def write_register(req):
    rospy.loginfo("Register update cb called")
    register_addr = todata(req.register)
    register_val = todata(req.value)
    data = motor_id + s_write + register_addr + register_val
    crc = crc16(data)
    data = data + crc
    ser.write(bytes(data))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            return WriteRegisterResponse(success=True)
        else:
            return WriteRegisterResponse(success=False)
    except:
        return WriteRegisterResponse(success=False)


def read_register(req):
    rospy.loginfo("Register read cb called")
    register_addr = todata(req.register)
    data = motor_id + m_read + register_addr + '\x00\x01'
    crc = crc16(data)
    data = data + crc
    ser.write(bytes(data))
    time.sleep(com_delay)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read()
        if crc_check(out):
            data1 = tohex(ord(out[3]), 16)
            data2 = tohex(ord(out[4]), 16)
            data = data1[2:] + data2[2:]
            data = int(data[-3:],16)
            return ReadRegisterResponse(success=True,value=data)
        else:
            return ReadRegisterResponse(success=False)
    except:
        return ReadRegisterResponse(success=False)


if __name__ == '__main__':
    rospy.init_node("servo_driver_"+motor_name,  anonymous=True)

    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1, parity=serial.PARITY_NONE,
                            bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
        connected = True
        rospy.loginfo("INFO:   Port  is Available")

    except:
        connected = False
        rospy.logerr("WARNING: Port " + port + " is Not Available")

    buttons_subs = rospy.Subscriber("/emergency", Bool, buttons_cb)
    enc_pub = rospy.Publisher("/motor_"+motor_name +
                              "/encoder", Int64, queue_size=10)
    alarm_pub = rospy.Publisher(
        "/motor_"+motor_name+"/alarm", String, queue_size=10)
    rospy.Service("/motor_"+motor_name + "/speed", Speed, status_cb)
    rospy.Service("/motor_"+motor_name+"/voltage", Voltage, voltage_cb)
    rospy.Service("/motor_"+motor_name+"/write_register",
                  WriteRegister, write_register)
    rospy.Service("/motor_"+motor_name+"/read_register",
                  ReadRegister, read_register)
    swri_rospy.Subscriber("/distributed_speed", String, cb)

    if connected:
        ser.flushInput()
        motor_check()
        swri_rospy.Timer(rospy.Duration(0.13), enc_cb) # Manish:duration changed to 0.033 from 0.13
        swri_rospy.Timer(rospy.Duration(1.1), alarm_cb) 
        atexit.register(exit_stop)
    swri_rospy.spin()
