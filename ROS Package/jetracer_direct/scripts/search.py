#!/usr/bin/env python3

import rospy
import time
from jetracer_direct.msg import motor_msg, strategy_msg

class Search():
	def __init__(self):
		rospy.init_node('search', anonymous=True)
		rospy.Subscriber('strategy_msg', strategy_msg, self.strategy_callback)
		pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		
		self.update_rate = 3
		self.update = time.time()
		self.strategy = []
		self.angle = 0
		while not rospy.is_shutdown():
			if ("search" in self.strategy):
				if (time.time() - self.update > self.update_rate):
					msg = motor_msg()
					msg.throttle = 0
					msg.steer = 90
					msg.swivel = self.angle
					pub.publish(msg)
					
					self.angle += 60
					if (self.angle > 180):
						self.angle = 0
					self.update = time.time()
		
	def strategy_callback(self, data):
		self.strategy = data.strategy

if __name__ == '__main__':
	node = Search()
