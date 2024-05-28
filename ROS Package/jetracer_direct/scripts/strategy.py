#!/usr/bin/env python3

import rospy
from enum import Enum
from jetracer_direct.msg import strategy_msg, objects_stamped_msg, objects_msg

class States(Enum):
	strike = 0
	search = 1
	flip = 2
	defense = 3
	
class StateMachine():
	def __init__(self):
		self.state = States.defense
		self.state_prev = None
		self.strategy = []
		self.pub = rospy.Publisher('strategy_msg', strategy_msg, queue_size=0)	
	
	def transition(self, recent_objects):
		'''
		ball: 4
		blue goal: 3
		red goal: 0
		cross: 2
		car: 1
		'''
		'''
		if (self.state == States.search):
			if ('2' in recent_objects):
				self.state = States.defense
			'''
		if (self.state == States.defense):
			if (('4' in recent_objects) and ('0' in recent_objects)):
				self.state = States.strike
		
		'''
		if(self.state == States.strike):
			if('4' not in recent_objects):
				self.state = States.search
			elif(('4' in recent_objects) and ('0' in recent_objects)):
				self.state = States.defense
		elif(self.state == States.search):
			if(('4' in recent_objects) and ('3' in recent_objects)):
				self.state = States.strike
			elif(('4' in recent_objects) and ('0' in recent_objects)):
				self.state = States.defense
		elif(self.state == States.flip):
			if (('4' in recent_objects) and ('3' in recent_objects)):
				self.state = States.strike
		elif(self.state == States.defense):
			self.state = States.flip
		'''
		
	def output(self):
		if(self.state == States.strike):
			self.strategy = ['strike']
		elif(self.state == States.search):
			self.strategy = ['search']
		elif(self.state == States.flip):
			self.strategy = ['flip']
		elif(self.state == States.defense):
			self.strategy = ['defense']
					
		msg = strategy_msg()
		msg.strategy = self.strategy
		self.pub.publish(msg)
		
		if(self.state_prev != self.state):
			self.state_prev = self.state
			print(str(self.state) + "...")

class Strategy:
	def __init__(self):
		sm = StateMachine()
		rospy.init_node('strategy', anonymous=True)
		rospy.Subscriber('objects_stamped_msg', objects_stamped_msg, self.msg_callback)
		
		self.recent_objects = []
		while not rospy.is_shutdown():
			sm.output()
			sm.transition(self.recent_objects)

	def msg_callback(self, data):
		self.recent_objects = data.recent_objects
		print(self.recent_objects)

if __name__ == '__main__':
	rospy.sleep(20) # important
	node = Strategy()
