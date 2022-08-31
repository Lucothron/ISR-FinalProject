#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class mainNode(Node):
	def __init__(self):
		super().__init__('PID')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
		self.ei = 0.0
		
	def PID(self):
		# -- PID constants -- 
		kp = 0.5
		ki = 0.01
		kd = 0.8
		ep = 0.0
		
		# Read minimum data from right sensor
		readSensor = min(self.laser_ranges[960:1200])
		
		self.get_logger().info('distance ' + str(readSensor))
		
		# Calculate errors
		e = 0.5 - readSensor
		self.ei += e
		ed = e - ep
		ep = e
		
		# Calculate output using PID function
		self.output = (kp * e) + (ki * self.ei) + (kd * ed)
		
		self.get_logger().info('output ' + str(self.output))
		
		return self.output
		
	def laser_callback(self, msg):
		#self.get_logger().info(str(msg.ranges[180]))
		self.laser_ranges = msg.ranges
		self.i += 1
	
	def timer_callback(self):
		if self.i > 0:
			self.cmd.linear.x = 0.25
			self.cmd.angular.z = self.PID()
			self.publisher_.publish(self.cmd)
		
def main(args=None):
	rclpy.init(args=args)
	
	initNode = mainNode()
	rclpy.spin(initNode)
	
	#Destroy node explicitly
	
	initNode.destroy_mode()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
