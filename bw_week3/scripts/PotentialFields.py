#!usr/bin/env python
import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Potential_Field():
    def __init__(self):
	self.driving = AckermannDriveStamped()
        self.fields_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        self.Lscan_sub = rospy.Subscriber("/scan", LaserScan, self.pfields_callback)

	self.a = 1.0
	self.b = -0.5

    def pfields_callback(self, msg):
	angle_increment = 3.63
	angle_min = 3
	ranges = 5.0
	self.charge_laser_particle = 100.0
	self.charge_forward_boost = 100.0
	self.boost_distance = 5.0

#	scan_rad_angles = ((msg.angle_increment * np.arange(100, dtype = float))) * msg.angle_min
	scan_rad_angles = []
	for i in range(0,len(msg.ranges)):
	    scan_rad_angles.append(msg.ranges[i])

	scan_x_unit_vectors = -np.cos(scan_rad_angles)
	scan_y_unit_vectors = -np.sin(scan_rad_angles)

	scan_x_components = (self.charge_laser_particle * scan_x_unit_vectors) / np.square(ranges)
	scan_y_components = (self.charge_laser_particle * scan_y_unit_vectors) / np.square(ranges)

	kick_x_components = np.ones(1) * self.charge_forward_boost / self.boost_distance**2.0
	kick_y_components = np.ones(1)

	total_x_components = np.sum(scan_x_components) + kick_x_components
	total_y_components = np.sum(scan_y_components) + kick_y_components

	self.motivator = 5

	speed = (self.a * ((sum(total_x_components)/len(total_x_components))**2 + (sum(total_y_components)/len(total_y_components))**2)**0.5 * np.sign(sum(total_x_components))) / 2000
	steering_angle = self.b * math.atan2(sum(total_y_components), sum(total_x_components)) * np.sign(sum(total_x_components))
#	print("Sumx ", sum(total_x_components), "  Sumy: ", sum(total_y_components))
	print("Speed: ", speed, "  and Angle: ", steering_angle)

	self.driving.drive.speed = speed
	self.driving.drive.steering_angle = steering_angle
	self.driving.header.stamp = rospy.Time.now()

	self.fields_pub.publish(self.driving)

rospy.init_node("PotentialFields")
controller = Potential_Field()
rospy.spin()
