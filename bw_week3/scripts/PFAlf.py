#!/usr/bin/env python
import rospy
import numpy as np
#from std_msgs.msg import Float32MultiArray
#from bw_week3.msg import speed as SpeedMsg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
import math

class Potential_Field_Alg():
     def __init__(self):
	self.driving = AckermannDriveStamped()
	#self.speed_pub = rospy.Publisher("/speeds", SpeedMsg, queue_size=10)
	
	self.SpeedP = .09
	self.AngleP = -1
	self.BackBlob = 100

	self.fconst = .2
	self.Lscan_sub = rospy.Subscriber("/scan", LaserScan, self.vectorAdd)
	self.PFAlgPub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)




     def vectorAdd(self, Lmsg):
	#smsg = SpeedMsg()

	xcoords = []
	ycoords =[]
	xsum = 0
	ysum = 0
	for i in range(0,len(Lmsg.ranges)):
	  force = self.fconst/(Lmsg.ranges[i]**2)
	  xcoords.append(force * math.cos(i*Lmsg.angle_increment + Lmsg.angle_min))
	  ycoords.append(force * math.sin(i*Lmsg.angle_increment + Lmsg.angle_min))
	xsum = sum(xcoords)	
	print(xsum)	
	xsum = self.BackBlob - xsum
	ysum = sum(ycoords)
	speed = self.SpeedP * ((xsum**2 + ysum**2)**.5)*np.sign(xsum)
	angle = self.AngleP * math.atan2(ysum, xsum)*np.sign(xsum)
	print('Speed: ', speed, '   Angle: ', angle)
	self.driving.drive.speed = speed
	self.driving.drive.steering_angle = angle
	self.driving.header.stamp = rospy.Time.now()
	
	#smsg.speed = speed
	#smsg.angle = angle
	#self.driving.header.stamp = rospy.Time.now()
	
	self.PFAlgPub.publish(self.driving)


rospy.init_node("PFAlg")
controller = Potential_Field_Alg()

rospy.spin()	

