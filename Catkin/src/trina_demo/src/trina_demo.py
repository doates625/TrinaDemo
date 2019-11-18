#!/usr/bin/env python

"""
demo_node.py
Baxter Embedded Sensor Control Demo Node
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from serial import Serial
from struct import pack, unpack
from time import sleep

"""
Serial Settings
"""
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200

"""
Main Function
"""
if __name__ == '__main__':

	# Init ROS node
	rospy.init_node('trina_demo')

	# Init baxter interface
	baxter_interface.RobotEnable(CHECK_VERSION).enable()
	arm = baxter_interface.Limb('left')
	joint_name = arm.joint_names()[6]
	
	# Open serial port
	serial = Serial(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
	
	# Control loop
	while not rospy.is_shutdown():
		
		# Signal MCU and wait for response
		serial.write(pack('B', 0x00)[0])
		while serial.in_waiting < 4:
			pass
		
		# Process response
		angle = unpack('f', serial.read(4))[0]
		arm.set_joint_positions({joint_name: angle})
		
		# Limit comm rate
		sleep(0.1)
