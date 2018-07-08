#!/usr/bin/python

# modules
import sys
import smbus
import time
import struct
import statistics

# consts
BUS_NUM			= 0x01			# I2C Bus
IMU_ADDR 		= 0x68			# Device ID
PWR_MGMT_1 		= 0x6B			# Power mgmt 1
CAL_MAX_SMPLS	= 0x400			# Number of samples for calibration
CAL_SMPL_RATE	= 0.001			# Sampling rate for calibration

ACCEL_XOUT_H	= 0x3B			# ACCEL X H
ACCEL_XOUT_L	= 0x3C			# ACCEL X L
ACCEL_YOUT_H	= 0x3D			# ACCEL Y H
ACCEL_YOUT_L	= 0x3E			# ACCEL Y L
ACCEL_ZOUT_H	= 0x3F			# ACCEL Z H
ACCEL_ZOUT_L	= 0x40			# ACCEL Z L

TEMP_OUT_H		= 0x41			# TEMP H
TEMP_OUT_L		= 0x42			# TEMP L

GYRO_XOUT_H		= 0x43			# GYRO X H
GYRO_XOUT_L		= 0x44			# GYRO X L
GYRO_YOUT_H		= 0x45			# GYRO Y H
GYRO_YOUT_L		= 0x46			# GYRO Y L
GYRO_ZOUT_H		= 0x47			# GYRO Z H
GYRO_ZOUT_L		= 0x48			# GYRO Z L

GYRO_CONFIG		= 0x1B			# Gyro config
ACCEL_CONFIG	= 0x1C			# Accelero config

GYRO_FS_250		= 0b00000000	# Full scale range +/- 250 deg/s
GYRO_FS_500		= 0b00001000	# Full scale range +/- 500 deg/s
GYRO_FS_1K		= 0b00010000	# Full scale range +/- 1000 deg/s
GYRO_FS_2K		= 0b00011000	# Full scale range +/- 2000 deg/s

ACCEL_FS2G		= 0b00000000	# Full scale range +/- 2g
ACCEL_FS4G		= 0b00001000	# Full scale range +/- 4g
ACCEL_FS8G		= 0b00010000	# Full scale range +/- 8g
ACCEL_FS16G		= 0b00011000	# Full scale range +/- 16g

LSB_PER_G		= {				# Sensitivity LSB/g
	ACCEL_FS2G: 16384.0,
	ACCEL_FS4G:  8192.0,
	ACCEL_FS8G:  4096.0,
	ACCEL_FS16G: 2048.0
}

# get access to the I2C bus
I2C_Bus = smbus.SMBus(BUS_NUM)

def getAcceleroInG(value, lsbpg):
	g = value / lsbpg
	return g;

def wakeupDevice():
	print 'Waking up MPU 6050 @{0:#02x}'.format(IMU_ADDR)
	I2C_Bus.write_byte_data(IMU_ADDR, PWR_MGMT_1, 0)
	return;

def readWord(bus, addr, reg_H, reg_L):
	value = (bus.read_byte_data(addr, reg_H) << 8) | bus.read_byte_data(addr, reg_L)
	return value;
	
def short2Signed(number):
	value = struct.unpack('h', struct.pack('H', number))[0]
	return value;
	
def config(accel_fs, gyro_fs):
	print 'Configuring: ACCEL={0:08b} GYRO={1:08b}'.format(accel_fs, gyro_fs)
	I2C_Bus.write_byte_data(IMU_ADDR, ACCEL_CONFIG, accel_fs)
	I2C_Bus.write_byte_data(IMU_ADDR, GYRO_CONFIG, gyro_fs)
	return;
	
def getAcceleroMeasurements():
	x = readWord(I2C_Bus, IMU_ADDR, ACCEL_XOUT_H, ACCEL_XOUT_H + 1)
	y = readWord(I2C_Bus, IMU_ADDR, ACCEL_YOUT_H, ACCEL_YOUT_H + 1)
	z = readWord(I2C_Bus, IMU_ADDR, ACCEL_ZOUT_H, ACCEL_ZOUT_H + 1)
	return (x, y, z);
	
def getGyroscopeMeasurements():
	x = readWord(I2C_Bus, IMU_ADDR, GYRO_XOUT_H, GYRO_XOUT_H + 1)
	y = readWord(I2C_Bus, IMU_ADDR, GYRO_YOUT_H, GYRO_YOUT_H + 1)
	z = readWord(I2C_Bus, IMU_ADDR, GYRO_ZOUT_H, GYRO_ZOUT_H + 1)
	return (x, y, z);

def getTemperature():
	t = readWord(I2C_Bus, IMU_ADDR, TEMP_OUT_H, TEMP_OUT_H + 1)
	t = short2Signed(t) / 340.0 + 36.53
	return t;

def calibrate():
	count = 0
	xCol = []
	yCol = []
	zCol = []
	
	print 'Start collecting data for calibration..'
	
	# sample n values
	while count < CAL_MAX_SMPLS:
		x, y, z = getAcceleroMeasurements()
		xCol.append(x)
		yCol.append(y)
		zCol.append(z)
		count = count + 1
		time.sleep(CAL_SMPL_RATE)
		
	# calculate stats
	meanX = statistics.mean(xCol)
	meanY = statistics.mean(yCol)
	meanZ = statistics.mean(zCol)

	medX = statistics.median(xCol)
	medY = statistics.median(yCol)
	medZ = statistics.median(zCol)
	
	stdX = statistics.stdev(xCol)
	stdY = statistics.stdev(yCol)
	stdZ = statistics.stdev(zCol)
	
	offsetX, offsetY, offsetZ = (-medX, -medY, -medZ)
	
	print 'Calibrated results: '
	print 'Mean:   {0:.3f}, {1:.3f}, {2:.3f}'.format(meanX, meanY, meanZ)
	print 'Median: {0:.3f}, {1:.3f}, {2:.3f}'.format(medX, medY, medZ)
	print 'Stdev:  {0:.3f}, {1:.3f}, {2:.3f}'.format(stdX, stdY, stdZ)
	print 'Offsets: {0}'.format((offsetX, offsetY, offsetZ))
		
	return (offsetX, offsetY, offsetZ);

# setup

print 'Working with MPU 6050 (GY-521 Module)'

# wake the device
wakeupDevice()
time.sleep(.10)

# configure it
gyro_fs = GYRO_FS_250
accel_fs = ACCEL_FS2G
config(accel_fs, gyro_fs)
time.sleep(.10)

# calibrate
ox, oy, oz = calibrate()
time.sleep(5)

# LSB/g
lsbpg = LSB_PER_G[accel_fs]

# let's play
print 'Reading data..'
while True:
	try:
		print '=== === === ==='
		x, y, z = getAcceleroMeasurements()
		x = getAcceleroInG(x + ox, lsbpg)
		y = getAcceleroInG(y + oy, lsbpg)
		z = getAcceleroInG(z + oz, lsbpg)
		
		t = getTemperature()
		
		print 'ACCEL({0:.3f}, {1:.3f}, {2:.3f})'.format(x, y, z)
		print 'TEMP: {0:.2f} deg C'.format(t)
		
		time.sleep(0.500)
	except:
		print 'Exiting: ', sys.exc_info()[:2]
		break

