#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import Event
from std_msgs.msg import Int32

from time import time

class Subscriber(Node):
	def __init__(self):
		super().__init__("motor_failure_node")
		qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
		self.subscription = self.create_subscription(Event, "/fmu/out/event", self.listener_callback, qos_profile)
		self.motor_failure_subscription = self.create_subscription(Int32, "/motor_failure/motor_number", self.motor_failure_listener, qos_profile)
		self.motor_failed = None
		self.t1 = None
		self.t2 = None
	def listener_callback(self, event_msg):
		if event_msg.arguments[0] == 0:
			return
		new_motor = True if self.motor_failed != event_msg.arguments[0] else False
		self.motor_failed = event_msg.arguments[0]
		if self.t2 == None and new_motor:
			self.t2 = time()
			print(f"Inference Time = {self.t2-self.t1} seconds")
		print(f"Motor Failed = {self.motor_failed}")
		exit(0)
	def motor_failure_listener(self, msg):
		self.t1 = time()

def main(args=None):
	rclpy.init(args=args)
	subscriber = Subscriber()
	try:
		rclpy.spin(subscriber)
	except KeyboardInterrupt:
		pass
	except Exception as error:
		print(f"Error Occured : {error}")
	subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
