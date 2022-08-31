#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class mainNode(Node):
	def __init__(self):
		super().__init__('OA_FLC')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 0.4
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
		
	def obstAvoid(self):

		# Function to calculate membership values
		def memVals(dir, bound, x):
			if dir == 'r':
				if len(bound) == 2:		# If the value is in a fuzzy set, calculate rising and falling values
					if bound == 'cm':
						self.memR[0] = (x - self.close)/(self.med - self.close) #rising
						self.memR[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memR[0] = (x - self.med)/(self.far - self.med) #rising
						self.memR[1] = (self.far - x)/(self.far - self.med) #falling
				else:					# If the value is on a flat set, membership value is 1
					self.memR[0] = 1

			elif dir == 'm':
				if len(bound) == 2:
					if bound == 'cm':
						self.memM[0] = (x - self.close)/(self.med - self.close) #rising
						self.memM[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memM[0] = (x - self.med)/(self.far - self.med) #rising
						self.memM[1] = (self.far - x)/(self.far - self.med) #falling
				else:
					self.memM[0] = 1
			
			elif dir == 'l':
				if len(bound) == 2:
					if bound == 'cm':
						self.memL[0] = (x - self.close)/(self.med - self.close) #rising
						self.memL[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memL[0] = (x - self.med)/(self.far - self.med) #rising
						self.memL[1] = (self.far - x)/(self.far - self.med) #falling
				else:
					self.memL[0] = 1

		firedRules = []
		ruleNums = []
		memValComb = []

		# Input boundaries
		self.close = 0.15
		self.med = 0.5
		self.far = 0.85

		# Ruleset
		self.rules = ['cccsr', 'ccmsr', 'ccfsr', 'cmcsr', 'cmmsr', 'cmfmr', 'cfcms', 'cfmsr', 'cffsr', 
		'mccsl', 'mcmsr', 'mcfsr', 'mmcsl', 'mmmss', 'mmfsr', 'mfcsl', 'mfmss', 'mffmr', 
		'fccsl', 'fcmsl', 'fcfsr', 'fmcml', 'fmmml', 'fmfsr', 'ffcsl', 'ffmml', 'fffms']

		# Read minimum values for right, front, and left sensor ranges
		self.rs = min(self.laser_ranges[1240:1380])
		self.ms = min(min(self.laser_ranges[0:60]), min(self.laser_ranges[1380:1440]))
		self.ls = min(self.laser_ranges[60:200])

		self.get_logger().info("Input values: " + str(self.rs) + " / " + str(self.ms) + " / " + str(self.ls))

		# Calculate boundaries of each read value
		if self.rs < self.close:
			self.rsBound = 'c'
		elif self.rs > self.close and self.rs < self.med:
			self.rsBound = 'cm'
		elif self.rs == self.med:
			self.rsBound = 'm'
		elif self.rs > self.med and self.rs < self.far:
			self.rsBound = 'mf'
		else:
			self.rsBound = 'f'		
		
		if self.ms < self.close:
			self.msBound = 'c'
		elif self.ms > self.close and self.ms < self.med:
			self.msBound = 'cm'
		elif self.ms == self.med:
			self.msBound = 'm'
		elif self.ms > self.med and self.ms < self.far:
			self.msBound = 'mf'
		else:
			self.msBound = 'f'	

		if self.ls < self.close:
			self.lsBound = 'c'
		elif self.ls > self.close and self.ls < self.med:
			self.lsBound = 'cm'
		elif self.ls == self.med:
			self.lsBound = 'm'
		elif self.ls > self.med and self.ls < self.far:
			self.lsBound = 'mf'
		else:
			self.lsBound = 'f'	

		# Create membership value arrays based on boundaries
		self.memR = [None] * len(self.rsBound)
		self.memM = [None] * len(self.msBound)
		self.memL = [None] * len(self.lsBound)

		# Call membership value function for each sensor
		memVals('r', self.rsBound, self.rs)
		memVals('m', self.msBound, self.ms)
		memVals('l', self.lsBound, self.ls)

		# Calculate combinations and log fired rules
		for i in range(len(self.lsBound)):
			for j in range(len(self.msBound)):
				for k in range(len(self.rsBound)):
					memValComb.append([self.memL[i], self.memM[j], self.memR[k]])
					firedRules.append(self.lsBound[i] + self.msBound[j] + self.rsBound[k])

		self.get_logger().info("Rule fired: " + str(firedRules))

		# Log number of rule fired and save index for future use
		for x in range(27):
			tempRule = self.rules[x]
			for y in range(len(firedRules)):
				if firedRules[y] == tempRule[0:3]:
					ruleNums.append(x)

		# Reset sum variables
		sumMins = 0
		outX = 0
		outZ = 0

		# Calculate output values
		for i in range(len(ruleNums)):
			ruleIndex = ruleNums[i]
			ruleTemp = self.rules[ruleIndex]		# Save fired rule to check output parameters
			self.get_logger().info("Rule fired: " + str(ruleTemp))
			minVal = min(memValComb[i])				# Calculate minimum of membership values for each combination
			sumMins += minVal						# Acumulate minimum values
			self.get_logger().info("MemVal: " + str(memValComb[i]))

			# Based on rule, accumulate final output
			if ruleTemp[3] == 's':
				outX += 0.1 * minVal
			elif ruleTemp[3] == 'm':
				outX += 0.3 * minVal

			if ruleTemp[4] == 'l':
				outZ += 0.7 * minVal
			elif ruleTemp[4] == 's':
				outZ += 0 * minVal
			elif ruleTemp[4] == 'r':
				outZ += -0.7 * minVal

			self.get_logger().info("Out X: " + str(outX))
			self.get_logger().info("Out Z: " + str(outZ))
			
		# Return accumulated sum of output by accumulated membership values
		return((outX / sumMins),(outZ / sumMins))

	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		self.i += 1
	
	def timer_callback(self):
		if self.i > 0:
			dirVals = self.obstAvoid()
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
