#!/usr/bin/env python3
#JetRacer Defense utilizes the position and depth data from the ZED2 camera to drive towards
# a cross located next to the JetRacer car's own goal and stop with enough room to turn around

import rospy
import time
from jetracer_direct.msg import objects_stamped_msg, objects_msg, motor_msg, strategy_msg
from simple_pid import PID
from enum import Enum
import math

class States(Enum):
	defend = 0
	flip = 1
	
class StateMachine():
	def __init__(self):
		self.pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		self.state = States.defend
		self.throttle = 0
		self.steer = 90
		self.angle = 90
		
		self.throttle_prev = None
		self.steer_prev = None
		self.angle_prev = None
		
		self.once = True
		
		kP = 0.025
		kI = 0
		kD = 0
		self.pid = PID(kP, kI, kD, setpoint = 700)

	def transition(self, distance):
		if(self.state == States.defend):
			if(distance <= 1):
				self.state = States.flip

		elif(self.state == States.flip):
			pass

	#Ensure published angle is within bounds 
	def angle_limit(self, angle):
		if(angle > 180):
			return 180
		elif(angle < 0):
			return 0
		else:
			return angle

	#Ensure published steering value is within bounds 
	def steer_rect_limit(self, steer):
		steer = 180 - steer
		if(steer > 120):
			return 120
		elif(steer < 60):
			return 60
		else:
			return steer
						
	#Determine correct steering and camera angles, and throttle value from provided error
	def output(self, error):
		if(self.state == States.defend):
			if ((error < 800) and (error > 600)):
				self.throttle = 0.15
			else:
				self.throttle = 0
			
			self.angle += int(self.pid(error))
			self.angle = self.angle_limit(self.angle)
			self.steer = self.steer_rect_limit(self.angle)
			
		elif(self.state == States.flip):
			self.throttle = 0.15
			self.angle = 180
			self.steer = 60

		print("Error: " + str(error) + ", Throttle: " + str(round(self.throttle, 2)) + ", Steer: " + str(self.steer) + ", Angle: " + str(self.angle) + ", " + str(self.state))
				
		if(self.throttle != self.throttle_prev or self.steer != self.steer_prev or self.angle != self.angle_prev):	
			msg = motor_msg()
			msg.throttle = self.throttle
			msg.steer = self.steer
			msg.swivel = self.angle
			self.pub.publish(msg)
			
			self.throttle_prev = self.throttle
			self.steer_prev = self.steer
			self.angle_prev = self.angle

#Class that controls JetRacer defensive strategy
class Defense():
	def __init__(self):
		sm = StateMachine()
		rospy.init_node('defense', anonymous=True)
		rospy.Subscriber('objects_stamped_msg', objects_stamped_msg, self.objects_stamped_callback)
		rospy.Subscriber('strategy_msg', strategy_msg, self.strategy_callback)
		self.pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		
		self.angle = 90
		self.steer = 90
		self.throttle = 0
		self.strategy = []
		self.center = 700
		self.distance = 5
		self.updated = False
		while not rospy.is_shutdown():
			if ("defense" in self.strategy):
				if (self.updated):		
					sm.output(self.center)
					sm.transition(self.distance)
					self.updated = False

	#Callback to receive data from CV model
	def objects_stamped_callback(self, data):
		lock = False
		for object in data.objects:
			if (object.label == '2' and lock == False):
				self.center = object.center
				self.distance = object.distance
				self.updated = True
				lock = True
	
	#Callback to receive data from strategy topic	
	def strategy_callback(self, data):
		self.strategy = data.strategy

if __name__ == '__main__':
	node = Defense()
