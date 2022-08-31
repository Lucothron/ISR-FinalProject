#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class mainNode(Node):
	def __init__(self):
		super().__init__('REF_FLC')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 0.4
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)

	def rightEdgeFollow(self):

		# Function to calculate membership values
		def memVals(dir, bound, x):
			if dir == 'f':
				if len(bound) == 2: 	# If the value is in a fuzzy set, calculate rising and falling values
					if bound == 'cm':
						self.memF[0] = (x - self.close)/(self.med - self.close) #rising
						self.memF[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memF[0] = (x - self.med)/(self.far - self.med) #rising
						self.memF[1] = (self.far - x)/(self.far - self.med) #falling
				else:					# If the value is on a flat set, membership value is 1
					self.memF[0] = 1

			elif dir == 'b':
				if len(bound) == 2:
					if bound == 'cm':
						self.memB[0] = (x - self.close)/(self.med - self.close) #rising
						self.memB[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memB[0] = (x - self.med)/(self.far - self.med) #rising
						self.memB[1] = (self.far - x)/(self.far - self.med) #falling
				else:
					self.memB[0] = 1
		
		firedRules = []
		ruleNums = []
		memValComb = []
		
		#Input boundaries
		self.close = 0.3
		self.med = 0.5
		self.far = 0.7

		# Ruleset
		self.rules = ['ccml', 'cmml', 'cfsl', 'mcmr', 'mmms', 'mfml', 'fcsr', 'fmmr', 'ffmr']
		
		# Read minimum values for right front and right back sensors
		self.rfs = min(self.laser_ranges[1100:1300])
		self.rbs = min(self.laser_ranges[860:1060])

		self.get_logger().info("Input values: " + str(self.rfs) + " / " + str(self.rbs))
	
		# Calculate boundaries of each read value
		if self.rfs < self.close:
			self.rfsBound = 'c'
		elif self.rfs > self.close and self.rfs < self.med:
			self.rfsBound = 'cm'
		elif self.rfs == self.med:
			self.rfsBound = 'm'
		elif self.rfs > self.med and self.rfs < self.far:
			self.rfsBound = 'mf'
		else:
			self.rfsBound = 'f'
			
		if self.rbs < self.close:
			self.rbsBound = 'c'
		elif self.rbs > self.close and self.rbs < self.med:
			self.rbsBound = 'cm'
		elif self.rbs == self.med:
			self.rbsBound = 'm'
		elif self.rbs > self.med and self.rbs < self.far:
			self.rbsBound = 'mf'
		else:
			self.rbsBound = 'f'

		# Create membership value arrays based on boundaries
		self.memF = [None] * len(self.rfsBound)
		self.memB = [None] * len(self.rbsBound)
			
		# Call membership value function for each sensor
		memVals('f', self.rfsBound, self.rfs)
		memVals('b', self.rbsBound, self.rbs)

		# Calculate combinations and log fired rules
		for i in range(len(self.rfsBound)):
			for j in range(len(self.rbsBound)):
				memValComb.append([self.memF[i], self.memB[j]])
				firedRules.append(self.rfsBound[i] + self.rbsBound[j])
			
		self.get_logger().info("Fired rules: " + str(firedRules))

		# Log number of rule fired and save index for future use
		for x in range(9):
			tempRule = self.rules[x]
			for y in range(len(firedRules)):
				if firedRules[y] == tempRule[0:2]:
					ruleNums.append(x)

		# Reset sum variables
		sumMins = 0
		outX = 0
		outZ = 0
		
		self.get_logger().info("Rules: " + str(ruleNums))

		# Calculate output values
		for i in range(len(ruleNums)):
			ruleIndex = ruleNums[i]
			ruleTemp = self.rules[ruleIndex]		# Save fired rule to check output parameters
			self.get_logger().info("Rule fired: " + str(ruleTemp))
			minVal = min(memValComb[i])				# Calculate minimum of membership values for each combination
			sumMins += minVal						# Acumulate minimum values
			self.get_logger().info("MemVal: " + str(memValComb[i]))

			# Based on rule, accumulate final output
			if ruleTemp[2] == 's':
				outX += 0.1 * minVal
			elif ruleTemp[2] == 'm':
				outX += 0.3 * minVal

			if ruleTemp[3] == 'l':
				outZ += 0.5 * minVal
			elif ruleTemp[3] == 's':
				outZ += 0 * minVal
			elif ruleTemp[3] == 'r':
				outZ += -0.5 * minVal

			self.get_logger().info("Out Z: " + str(outZ))

		# Return accumulated sum of output by accumulated membership values
		return((outX / sumMins),(outZ / sumMins))
		
	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		self.i += 1
	
	def timer_callback(self):
		if self.i > 0:
			dirVals = self.rightEdgeFollow()
			self.cmd.linear.x = dirVals[0]
			self.cmd.angular.z = dirVals[1]
			self.publisher_.publish(self.cmd)
			self.get_logger().info("Output values: " + str(dirVals))
			self.get_logger().info("#####################################################################")
		
def main(args=None):
	rclpy.init(args=args)
	
	initNode = mainNode()
	rclpy.spin(initNode)
	
	#Destroy node explicitly
	
	initNode.destroy_mode()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()