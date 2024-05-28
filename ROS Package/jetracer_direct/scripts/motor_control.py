#!/usr/bin/env python3

# Libraries
import rospy
from adafruit_servokit import ServoKit
from jetracer_direct.msg import motor_msg

# Motor class subscribes to the motor control message, which contains a throttle value, camera angle, and steering angle. 
# The adafruit_servokit library is then used to write these values to the appropriate location
class Motor():
	def __init__(self):
		self.kit = ServoKit(channels=16)
		rospy.init_node('motor_control', anonymous=True)
		rospy.Subscriber('motor_msg', motor_msg, self.msg_callback)
		rospy.spin()
		
	def msg_callback(self, data):
		self.kit.continuous_servo[0].throttle = data.throttle
		self.kit.servo[1].angle = data.steer
		self.kit.servo[2].angle = data.swivel

if __name__ == '__main__':
	node = Motor()
