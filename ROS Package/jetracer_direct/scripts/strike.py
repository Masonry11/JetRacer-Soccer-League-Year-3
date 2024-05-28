#!/usr/bin/env python3

import rospy
import time
from jetracer_direct.msg import objects_stamped_msg, objects_msg, motor_msg, strategy_msg
from simple_pid import PID
from enum import Enum
import math

class States(Enum):
	strike = 0
	reverse = 1
	
class StateMachine():
	def __init__(self):
		self.pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		self.state = States.reverse
		self.throttle = 0
		self.steer = 90
		self.angle = 180
		
		self.throttle_prev = None
		self.steer_prev = None
		self.angle_prev = None
		
		kP = 0.025
		kI = 0
		kD = 0
		self.pid = PID(kP, kI, kD, setpoint = 700)

	def transition(self, error):
		if(self.state == States.strike):
			if((self.angle < 10 and error > 750) or (self.angle > 170 and error < 650)):
				self.state = States.reverse

		elif(self.state == States.reverse):
			if ((error < 750) and (error > 650)):
				self.state = States.strike
				self.pid.reset()

	def angle_limit(self, angle):
		if(angle > 180):
			return 180
		elif(angle < 0):
			return 0
		else:
			return angle
			
	def steer_rect_limit(self, steer):
		steer = 180 - steer
		if(steer > 120):
			return 120
		elif(steer < 60):
			return 60
		else:
			return steer
						
	def output(self, error):
		if(self.state == States.strike):
			if ((error < 800) and (error > 600)):
				self.throttle = 0.18
			else:
				self.throttle = 0

			self.angle += int(self.pid(error))
			self.angle = self.angle_limit(self.angle)
			self.steer = self.steer_rect_limit(self.angle-20)
			
		elif(self.state == States.reverse):
			self.angle = 90
			self.throttle = -0.07
			if (error > 700):
				self.steer = 60
			else:
				self.steer = 120

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


class Strike():
	def __init__(self):
		sm = StateMachine()
		rospy.init_node('strike', anonymous=True)
		rospy.Subscriber('objects_stamped_msg', objects_stamped_msg, self.objects_stamped_callback)
		rospy.Subscriber('strategy_msg', strategy_msg, self.strategy_callback)
		self.pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		
		self.angle = 90
		self.steer = 90
		self.throttle = 0
		self.strategy = []
		self.ball_position = [0, 1, 0]
		self.updated = True
		self.center = 700
		self.updated = False
		while not rospy.is_shutdown():
			if ("strike" in self.strategy):
				if (self.updated):		
					sm.output(self.center)
					sm.transition(self.center)
					self.updated = False

	def objects_stamped_callback(self, data):
		lock = False
		for object in data.objects:
			if (object.label == '4' and lock == False):
				self.ball_position = object.position
				self.center = object.center
				self.updated = True
				lock = True
		
	def strategy_callback(self, data):
		self.strategy = data.strategy

if __name__ == '__main__':
	node = Strike()
