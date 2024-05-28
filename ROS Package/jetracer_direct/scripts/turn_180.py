#!/usr/bin/env python3

import rospy
import sys
import time
from jetracer_direct.msg import motor_msg, strategy_msg

'''
global throttle
global angle
global steer

def reverse(angle, steer):
	if(steer == 60):
		return 180, 120
	else:
		return 0, 60 


#def msg_callback():
#	global INVERT 
#	INVERT = invert

#def init():


def pub_motor_msg(throttle, steer, angle):
	msg = motor_msg()
	msg.throttle = throttle 
	msg.steer = steer
	msg.swivel = angle
	print(angle, steer, throttle)
	pub.publish(msg)
	 
def turn():
	print('turning')
	global throttle
	global angle
	global steer

#	if(INVERT):
#		angle, steer = reverse(angle, steer)
	throttle = -0.1
	pub_motor_msg(throttle, steer, angle)
	rospy.sleep(1)
	angle, steer = reverse(angle, steer)
	throttle = 0.3
	pub_motor_msg(throttle, steer, angle)
	rospy.sleep(1)
	throttle = 0.05
	pub_motor_msg(throttle, steer, angle)
	rospy.sleep(0.5)
'''
	
class Flip():
	def __init__(self):
		rospy.init_node('turn_180', anonymous=True)
		rospy.Subscriber('strategy_msg', strategy_msg, self.strategy_callback)
		pub = rospy.Publisher('motor_msg', motor_msg, queue_size=0)
		
		self.strategy = []
		while not rospy.is_shutdown():
			if ("flip" in self.strategy):
				pass
		
	def strategy_callback(self, data):
		self.strategy = data.strategy

if __name__ == '__main__':
	node = Flip()
	'''
	angle = 0
	steer = 60  
	throttle = 0
#	init()
	while(1):
		turn()
		rospy.sleep(7)
	'''
