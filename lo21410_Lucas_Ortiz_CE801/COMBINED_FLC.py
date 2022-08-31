#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import OA_FLC
import REF_FLC

class mainNode(Node):
	def __init__(self):
		super().__init__('COMBINED_FLC')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 0.4
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)

	def combineFLC(self):

		# Input boundaries
		self.OA = 0.5
		self.REF = 0.9

		# "Ruleset" xD
		self.rules = ['orx']

		# Calculate minimum value between all three front sensors
		self.rs = min(self.laser_ranges[1240:1380])
		self.ms = min(min(self.laser_ranges[0:60]), min(self.laser_ranges[1380:1440]))
		self.ls = min(self.laser_ranges[60:200])

		self.minVal = min(self.rs, self.ms, self.ls) 		# Final minimum value

		self.get_logger().info("Val: " + str(self.minVal))

		# Calculate boundaries of read value
		if self.minVal < self.OA:
			self.bound = 'o'
		elif self.minVal > self.OA and self.minVal < self.REF:
			self.bound = 'or'
		else:
			self.bound = 'r'

		# Create membership value arrays based on boundaries
		self.mem = [None] * len(self.bound)

		# Calculate membership values 
		if len(self.bound) == 2:
			self.mem[0] = (self.minVal - self.OA)/(self.REF - self.OA) #rising
			self.mem[1] = (self.REF - self.minVal)/(self.REF - self.OA) #falling
		else:
			self.mem[0] = 1

		self.get_logger().info("Boundaries: " + str(self.bound))
		self.get_logger().info("Membership V: " + str(self.mem))

		# Calculate output values
		if self.bound == 'o':			# If obstacles directly in front, only use obstacle avoidance
			outVals = self.obstAvoid()
		elif self.bound == 'r':			# If no obstacles, only use right edge follow
			outVals = self.rightEdgeFollow()
		if len(self.bound) == 2:		# If obstacles in front, calculate fuzzy output
			memSum = self.mem[0] + self.mem[1]		# Sum of membership values
			OAval = self.obstAvoid()				# Get obstacle avoidance values
			REFval = self.rightEdgeFollow()			# Get right edge follow values
			self.get_logger().info("OA: " + str(OAval))
			for i in range(2):						# Multiply each value by membership value
				OAval[i] = OAval[i] * self.mem[0]
				REFval[i] = REFval[i] * self.mem[1]

			finX = (OAval[0] + REFval[0]) / memSum		# Add values and divide by total membership value
			finZ = (OAval[1] + REFval[1]) / memSum
			outVals = ([finX, finZ])

		# Return final values
		return(outVals)

	def rightEdgeFollow(self):

		def memVals(dir, bound, x):
			if dir == 'f':
				if len(bound) == 2:
					if bound == 'cm':
						self.memF[0] = (x - self.close)/(self.med - self.close) #rising
						self.memF[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memF[0] = (x - self.med)/(self.far - self.med) #rising
						self.memF[1] = (self.far - x)/(self.far - self.med) #falling
				else:
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

		self.rules = ['ccml', 'cmml', 'cfsl', 'mcmr', 'mmms', 'mfml', 'fcsr', 'fmmr', 'ffmr']
		
		self.rfs = min(self.laser_ranges[1100:1300])
		self.rbs = min(self.laser_ranges[860:1060])

		#self.get_logger().info("Input values: " + str(self.rfs) + " / " + str(self.rbs))
		
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

		self.memF = [None] * len(self.rfsBound)
		self.memB = [None] * len(self.rbsBound)
			
		memVals('f', self.rfsBound, self.rfs)
		memVals('b', self.rbsBound, self.rbs)

		for i in range(len(self.rfsBound)):
			for j in range(len(self.rbsBound)):
				memValComb.append([self.memF[i], self.memB[j]])
				firedRules.append(self.rfsBound[i] + self.rbsBound[j])
			
		#self.get_logger().info("Fired rules: " + str(firedRules))

		for x in range(9):
			tempRule = self.rules[x]
			for y in range(len(firedRules)):
				if firedRules[y] == tempRule[0:2]:
					ruleNums.append(x)

		# Reset sum variables
		sumMins = 0
		outX = 0
		outZ = 0
		
		#self.get_logger().info("Rules: " + str(ruleNums))
		#self.get_logger().info("Combinations: " + str(memValComb))

		for i in range(len(ruleNums)):
			ruleIndex = ruleNums[i]
			ruleTemp = self.rules[ruleIndex]
			#self.get_logger().info("Rule fired: " + str(ruleTemp))
			minVal = min(memValComb[i])
			sumMins += minVal
			#self.get_logger().info("MemVal: " + str(memValComb[i]))

			if ruleTemp[2] == 's':
				outX += 0.1 * minVal
			elif ruleTemp[2] == 'm':
				outX += 0.3 * minVal

			if ruleTemp[3] == 'l':
				outZ += 0.7 * minVal
			elif ruleTemp[3] == 's':
				outZ += 0 * minVal
			elif ruleTemp[3] == 'r':
				outZ += -0.7 * minVal

			#self.get_logger().info("Out Z: " + str(outZ))

		return([(outX / sumMins),(outZ / sumMins)])

	def obstAvoid(self):

		def memVals(dir, bound, x):
			if dir == 'r':
				if len(bound) == 2:
					if bound == 'cm':
						self.memR[0] = (x - self.close)/(self.med - self.close) #rising
						self.memR[1] = (self.med - x)/(self.med - self.close) #falling
					elif bound == 'mf':
						self.memR[0] = (x - self.med)/(self.far - self.med) #rising
						self.memR[1] = (self.far - x)/(self.far - self.med) #falling
				else:
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
		self.close = 0.2
		self.med = 0.5
		self.far = 0.8

		self.rules = ['cccsr', 'ccmsr', 'ccfsr', 'cmcsr', 'cmmsr', 'cmfmr', 'cfcms', 'cfmsr', 'cffsr', 
		'mccsl', 'mcmsr', 'mcfsr', 'mmcsl', 'mmmss', 'mmfsr', 'mfcsl', 'mfmss', 'mffmr', 
		'fccsl', 'fcmsl', 'fcfsr', 'fmcml', 'fmmml', 'fmfsr', 'ffcsl', 'ffmml', 'fffms']

		self.rs = min(self.laser_ranges[1240:1380])
		self.ms = min(min(self.laser_ranges[0:60]), min(self.laser_ranges[1380:1440]))
		self.ls = min(self.laser_ranges[60:200])

		#self.get_logger().info("Input values: " + str(self.rs) + " / " + str(self.ms) + " / " + str(self.ls))

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

		self.memR = [None] * len(self.rsBound)
		self.memM = [None] * len(self.msBound)
		self.memL = [None] * len(self.lsBound)

		memVals('r', self.rsBound, self.rs)
		memVals('m', self.msBound, self.ms)
		memVals('l', self.lsBound, self.ls)

		for i in range(len(self.lsBound)):
			for j in range(len(self.msBound)):
				for k in range(len(self.rsBound)):
					memValComb.append([self.memL[i], self.memM[j], self.memR[k]])
					firedRules.append(self.lsBound[i] + self.msBound[j] + self.rsBound[k])

		#self.get_logger().info("Rule fired: " + str(firedRules))

		for x in range(27):
			tempRule = self.rules[x]
			for y in range(len(firedRules)):
				if firedRules[y] == tempRule[0:3]:
					ruleNums.append(x)

		sumMins = 0
		outX = 0
		outZ = 0

		for i in range(len(ruleNums)):
			ruleIndex = ruleNums[i]
			ruleTemp = self.rules[ruleIndex]
			#self.get_logger().info("Rule fired: " + str(ruleTemp))
			minVal = min(memValComb[i])
			sumMins += minVal
			#self.get_logger().info("MemVal: " + str(memValComb[i]))

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

			#self.get_logger().info("Out X: " + str(outX))
			#self.get_logger().info("Out Z: " + str(outZ))
			
		return([(outX / sumMins),(outZ / sumMins)])

	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		self.i += 1
	
	def timer_callback(self):
		if self.i > 0:
			dirVals = self.combineFLC()
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