#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import time, collections


class pid_controller():
    def __init__(self):
        self.driving = AckermannDriveStamped()
        self.driving.header.stamp = rospy.Time.now()
        self.driving.drive.speed = 5
        self.ddes = .6
        self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
        self.kp = .5
        self.ki = .15
        self.kd = .05
        self.mult = -1  # left
        self.start_ind = 580  # left
        self.end_ind = 1000  # left
        self.side_sub = rospy.Subscriber("/racecar/JoyLRSelec", Bool, self.side_callback)
        self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.pid_callback)

    def pid_callback(self, msg):
        side = msg.ranges[self.start_ind:self.end_ind]
        dist = sum(sorted(side)[6:16])/10.0
	front = msg.ranges[500:580]
	fWarning = sum(front)/len(front)
	lTurn = sum(msg.ranges[440:460])/20.0
	rTurn = sum(msg.ranges[620:640])/20.0
	error = self.ddes - dist
        if fWarning < 1.25:
	    if 0 < (lTurn-rTurn): turnM= -1
	    else: turnM = 1
	    print (turnM,fWarning)
            self.driving.drive.steering_angle = .34*turnM
        elif abs(error)<.03:
		self.driving.drive.steering_angle = 0
	else:
            self.driving.drive.steering_angle = self.mult * self.pid(self.kp, self.kd, self.ki, error)
        self.pid_pub.publish(self.driving)
	print "steer " + str(self.driving.drive.steering_angle)
    
    def side_callback(self, msg):
        if msg.data:  # left wall
            self.mult = -1
            self.start_ind = 580
            self.end_ind = 1000
        else:  # right wall
            self.mult = 1
            self.start_ind = 80
            self.end_ind = 500

    def pid(self, kp, kd, ki, error):
        prev_error = self.prev_errors.popleft()
        prev_time = self.prev_times.popleft()
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        self.prev_times.append(time.clock())
        self.prev_errors.append(error)
        return kp * error + kd * e_deriv + ki * e_int


rospy.init_node("pid")
controller = pid_controller()

rospy.spin()
