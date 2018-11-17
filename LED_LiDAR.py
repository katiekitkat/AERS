#!/usr/bin/env python
# IMPORT smbus, time AND RPi.GPIO LIBRARIES
import smbus
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# CREATE THE Lidar_Lite CLASS. CONSISTS OF THE READ AND WRITE I2C LOCATIONS AND THE CONNECTED, GET DISTANCE AND VELOCITY COMMANDS.
class Lidar_Lite():
	def _init_(self):
		self.address = 0x62
		self.distWriteReg = 0x00
		self.distWriteVal = 0x04
		self.distReadReg1 = 0x8f
		self.distReadReg2 = 0x10
		self.velWriteReg = 0x04
		self.velWriteVal = 0x08
		self.velReadReg = 0x09
	def connect(self, bus):
		try:
			self.bus = smbus.SMBus(bus)
			time.sleep(0.5)
			return 0
		except:
			return -1
	def writeAndWait(self, register, value):
		self.bus.write_byte_data(self.address, regiser, value);
		time.sleep(0.02)
	def readAndWait(self, register):
		res = self.bus.read_byte_data(self.address, register)
		time.sleep(0.02)
		return res
	def readDistAndWait(self, register):
		res = self.bus.read_i2c_block_data(self.address, register, 2)
		time.sleep(0.02)
		return (res[0] << 8 | res[1])
	def getDistance(self):
		self.writeAndWait(self.distWriteReg, self.distWriteVal)
		dist = self.readDistAndWait(self.distReadReg1)
		return dist
	def getVelocity(self):
		self.writeAndWait(self.distWriteReg, self.distWriteVal)
		self.writeAndWait(self.velWriteReg, self.velWriteVal)
		vel = self.readAndWait(self.velReadReg)
		return signedInt(vel)
	def signedInt(value):
		if value > 127:
			return (256-value) * (-1)
		else:
			return value

		
# ATTACH LED PINS (17, 22, 23, 27) AND TURN OFF (JUST INCASE THEY WERE ALREADY ON FOR ANY REASON)
GPIO.setup(17,GPIO.OUT) # RED
GPIO.setup(22,GPIO.OUT) # GREEN
GPIO.setup(23,GPIO.OUT) # BLUE
GPIO.setup(27,GPIO.OUT) # YELLOW
GPIO.output(17,GPIO.LOW)
GPIO.output(22,GPIO.LOW)
GPIO.output(23,GPIO.LOW)
GPIO.output(27,GPIO.LOW)

# CHECK LIDAR IS CONNECTED AND TURN ON BLUE LIGHT IF SO
lidar = Lidar_Lite()
connected = lidar.connect(1)
if connected < -1:
	print("NOT CONNECTED")
else:
	print("CONNECTED")
	GPIO.output(23,GPIO.HIGH)		# TURNING ON BLUE LIGHT TO SAY LIDAR IS CONNECTED

# SETUP LIDAR CONTROL SERVO (PIN 12, 50HZ)
GPIO.setup(12,GPIO.OUT)
p = GPIO.PWM(12,50)

# SETUP FOR STEERING SERVO (BIG_ONE)
# GPIO.setup(BIG_ONE_PIN,GPIO.OUT)
# q = GPIO.PWM(BIG_ONE_PIN,50)

# CODE FOR ESC

# MATHS FOR LIDAR RESULTS
# def lidarmath(l):
# return (l*1000)

#THE BIG LOOP!
while true:
	autoset = raw_input("ENTER 'auto' FOR AUTONOMOUS MODE OR 'manual' FOR MANUAL MODE:")
	x = 0.7
	while (autoset != auto) or (autoset != manual):
		print ("INCORRECT INPUT!")
		autoset = raw_input("ENTER 'auto' FOR AUTONOMOUS MODE OR 'manual' FOR MANUAL MODE:")
	
	# MANUAL MODE
	while autoset.strip() == 'manual':
		distance = lidar.getDistance()
		print("DISTANCE TO THE NEAREST OBJECT IS = %s" % (distance))
		if (distance) < (x/4):
			print("OBJECT TOO CLOSE!")
			while int(distance) < (x/4):
				GPIO.output(17,GPIO.HIGH)
				p.start(10)
				print("REVERSING")
				p.start(7.5)
				time.sleep(0.2)
				GPIO.output(17,GPIO.LOW)
		p.stop(10)
		p.stop(7.5)
		
		elif (distance) < x:
			print("OBJECT LESS THAN 0.7 METERS AWAY!")
			while int(distance) < x:
				GPIO.output(27,GPIO.HIGH)
				time.sleep(0.5)
				GPIO.output(27,GPIO.LOW)
		
		else:
			GPIO.output(22,GPIO.HIGH)
			time.sleep (0.5)
			GPIO.output(22,GPIO.LOW)
		modech1 = raw_input("CHANGE MODE?")
		if modech1.strip() == 'YES':
			break
			
			
			
	# AUTONOMOUS MODE
	while autoset.strip() == 'auto':
		distance = lidar.getDistance()
		print("DISTANCE TO THE NEAREST OBJECT IS = %s" % (distance))
		if (distance) < (x/4):
			print("OBJECT TOO CLOSE!")
			while int(distance) < (x/4):
				GPIO.output(17,GPIO.HIGH)
				p.start(10)
				print("REVERSING")
				p.start(7.5)
				time.sleep(0.2)
				GPIO.output(17,GPIO.LOW)
		p.stop(10)
		p.stop(7.5)
		
		elif (distance) < x:
			print("OBJECT LESS THAN 0.7 METERS AWAY, SLOWING DOWN")
			while int(distance) < x:
				GPIO.output(27,GPIO.HIGH)
				time.sleep(0.5)
				GPIO.output(27,GPIO.LOW)
		
		else:
			GPIO.output(22,GPIO.HIGH)
			time.sleep(0.5)
			GPIO.output(22,GPIO.LOW)
		modech2 = raw_input("CHANGE MODE?")
		if modech2.strip() == 'YES':
			break
	
# TURN OFF CONDITIONS
GPIO.output(17,GPIO.LOW)
GPIO.output(22,GPIO.LOW)
GPIO.output(23,GPIO.LOW)
GPIO.output(27,GPIO.LOW)
