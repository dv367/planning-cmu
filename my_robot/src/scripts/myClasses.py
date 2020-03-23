freq = 100
class PID:
	kp = 0
	kd = 0
	ki = 0
	required = 0 
	error = 0 
	prevError = 0
	derivative = 0
	integral = 0
	output = 0
	minControl = 0
	maxControl = 0

	def __init__(self,kp,kd,ki,required,minControl,maxControl):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.required = required
		self.maxControl = maxControl
		self.minControl = minControl

	def pidControl(self,actual):
		 
		self.error = self.required - actual
		self.derivative = (self.error - self.prevError)*freq
		self.integral = self.integral + self.error
		self.prevError = self.error
		self.output = self.kp*self.error + self.kd*self.derivative + self.ki*self.integral
		
		if self.output > self.maxControl:
			self.output = self.maxControl
		elif self.output < self.minControl:
			self.output = self.minControl
		return self.output
