#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
from bw_week3.msg import speed as SpeedMsg
from std_msgs.msg import Bool
import time, collections


class pid_controller():
    def __init__(self):
        self.ddes = .6
        self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
        self.kp = .5 #.5
        self.ki = .15 #.15
        self.kd = .05 #.05
        self.left_start_ind = 580  # left
        self.left_end_ind = 1000  # left
        self.right_start_ind = 80
        self.right_end_ind = 500
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.pid_callback)
	self.speed_pub = rospy.Publisher("/speeds", SpeedMsg, queue_size=10)

    def pid_callback(self, msg):
	smsg = SpeedMsg()
	leftside = msg.ranges[self.left_start_ind:self.left_end_ind]        
	rightside = msg.ranges[self.right_start_ind:self.right_end_ind]
	lDist = sum(sorted(leftside)[6:16])/10.0        
	rDist = sum(sorted(rightside)[6:16])/10.0
	front = msg.ranges[480:600]
	fWarning = sum(front)/len(front)
	lTurn = sum(msg.ranges[440:460])/20.0
	rTurn = sum(msg.ranges[620:640])/20.0
	error = lDist - rDist	
	if lDist>2:
	    error = self.ddes - rDist
	elif rDist>2:
	    error = self.ddes - lDist
	blocked = False
	stop =False
	block =0
	collisions=0
        for i in range(len(front)):
	    if front[i] < 1.1:
		block +=1
	        if front[i]<.75:
		    collisions+=1
    	    else:
		block =0
		collsions =0
	    if collisions >4:
		stop= True
		break
	    if block >4:
		blocked= True
		break
	if fWarning < 1.35 or blocked:
	    if 0 < (lTurn-rTurn): turnM= -1
	    else: turnM = 1
            smsg.angle = .5*turnM
        elif abs(error)<.03:
		smsg.angle = 0
	else:
            smsg.angle = self.pid(self.kp, self.kd, self.ki, error)
	smsg.speed = 5	
	if stop:
	    smsg.speed =-1
	print(smsg.speed,smsg.angle)
	self.speed_pub.publish(smsg)

    def pid(self, kp, kd, ki, error):
        prev_error = self.prev_errors.popleft()
        prev_time = self.prev_times.popleft()
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        self.prev_times.append(time.clock())
        self.prev_errors.append(error)
        return kp * error + kd * e_deriv + ki * e_int

if __name__=="__main__":
    rospy.init_node("pid")
    controller = pid_controller()
    rospy.spin()
