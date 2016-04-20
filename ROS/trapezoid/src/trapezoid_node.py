#!/usr/bin/env python
# trapezoid_node: ROS node to interface with trapezoid board.


# To be run in conjunction with Comm_Kalman.ino
# Arduino file

import serial
import time

# ros imports
import rospy
from geometry_msgs.msg import PoseStamped
from trapezoid.srv import *

# Check in Arduino environment what com port is used
# it is com7 on TV's computer. If on linux, ls /dev
ARDUINO_SERIAL_PORT = 'com7'
ARDUINO_BAUDRATE = 115200

# tx and rx to/from Arduino
tx = [0] * 16
rx = [0] * 16

# Data to be tx to Arduino
header
load
trigger
pitch
yaw
PWM

# Data to be rx from Arduino
kalAngleX
kalAngleY
kalAngleZ

# Constant to get more decimal places of float data from Arduino
# Set equal to floats
kalConstX = 100.0
kalConstY = 100.0
kalConstZ = 100.0

# Continuously writes to Arduino
# and read back from it
def main():
	# subscribers
	rospy.Subscriber('/trapezoid/turret_pose', PoseStamped, handle_turret_pose)

	# publishers
	pub_pose = rospy.Publisher('/trapezoid/pose', PoseStamped, queue_size=10)

	# services
	rospy.Service('/trapezoid/shoot', Shoot, handle_shoot)

	#init the node
	rospy.init_node('trapezoid', anonymous=True)


	arduinoData = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_BAUDRATE)
	while (1 == 1):
	    #To Do:
	    #write to arduino data
	    #when get aiming information
	    arduinoTX()
	    
	    #receive arduino data
	    if(arduinoData.inWaiting() > 0):
	        arduinoRX()
	        #todo: publish the pose


# Send information to arduino
def arduinoTX():
	# Split data into different packets
	header0 = (header >> 8) & 255
	header1 = header & 255

	pitch4 = (pitch >> 8) & 255
	pitch5 = pitch & 255

	yaw6 = (yaw >> 8) & 255
	yaw7 = yaw & 255

	PWM8 = (PWM >> 8) & 255
	PWM9 = PWM & 255

	# send the data
    tx[0] = header0
    tx[1] = header1
    tx[2] = load
    tx[3] = trigger
    tx[4] = pitch4
    tx[5] = pitch5
    tx[6] = yaw6
    tx[7] = yaw7
    tx[8] = PWM8
    tx[9] = PWM9
    arduinoData.write(bytearray(tx))
    

# receive information from arduino
def arduinoRX():
    myData = arduinoData.read(16)

    # change string representation or rx data to int
    for j in range(len(myData)):
        rx[j] = ord(myData[j])
        
    kalAngleX = (( (rx[2] << 8)) | (rx[3] & 255))
    kalAngleY = (( (rx[4] << 8)) | (rx[5] & 255))
    kalAngleZ = (( (rx[6] << 8)) | (rx[7] & 255))

    # to get correct negative representation of data in Python
    kalAngleX = twosComp(16, kalAngleX)
    kalAngleY = twosComp(16, kalAngleY)
    kalAngleZ = twosComp(16, kalAngleZ)
    
    kalAngleX = kalAngleX / kalConstX
    kalAngleY = kalAngleY / kalConstY
    kalAngleZ = kalAngleZ / kalConstZ

# -------- subscriber handlers --------
def handle_turret_pose(data):
	print data

# -------- service handlers --------
def handle_shoot(req):
	print req
	return req


# Change negative numbers to
# correct readable representation in Python
def twosComp(bits, value):
    #if value is negative
    if (value >> 15) == 1:
        value = ~value + 1
        return -(( 1<<bits ) + value)
    else:
        return value


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass