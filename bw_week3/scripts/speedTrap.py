#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from bw_week3.msg import speed as SpeedMsg

class speed_control():
    def __init__(self):
	self.driving = AckermannDriveStamped()
	self.speedy = rospy.Subscriber("/speeds", SpeedMsg, self.speedcomptroller)
	self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	self.previousspeed=0
	self.acc = .2

    def speedcomptroller(self,msg):
	self.driving.drive.steering_angle = msg.angle
	
	diff = (msg.speed - self.previousspeed)
	if diff > 0: mult=1
	else: mult = -1	
	if abs(diff) > self.acc:
	    self.driving.drive.speed = self.previousspeed + mult*self.acc
	else:
	    self.driving.drive.speed = msg.speed
	self.previousspeed = self.driving.drive.speed
	print (self.driving.drive.speed ,msg.angle)
	self.pid_pub.publish(self.driving)

if __name__ == '__main__':
    rospy.init_node('roadrunner')
    node = speed_control()
    rospy.spin()
