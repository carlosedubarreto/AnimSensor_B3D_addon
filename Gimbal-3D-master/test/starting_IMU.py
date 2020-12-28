import smbus
import math
import time

# Registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) #I2C port 1
address = 0x68 # This is the address value via the i2cdetect command or chip user guide

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_B(adr):
	low = bus.read_byte_data(address, adr)
	high = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val= read_word(adr)
	if (val >= 0x8000):
		return - ((65535 - val) + 1)
	else:
		return val

def read_word_3c(adr):
	val= read_word_B(adr)
	if (val >= 0x8000):
		return - ((65535 - val) + 1)
	else:
		return val 

#Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)
bus.write_byte_data(address, power_mgmt_2, 0)


#check mode of gyroscope and accelerometer
mode_a=bus.read_byte_data(address, 0x1c)
mode_g=bus.read_byte_data(address, 0x1b)
print mode_a, mode_g

#define scales
SCALEa = 65536.0/4
SCALEg = 131.0

i=0
while True:
	i = i+1
	print "------------"
	print "acceleromater data, cnt =%d" %i
	accel_xout = read_word_2c(0x3b)
	accel_yout = read_word_2c(0x3d)
	accel_zout = read_word_2c(0x3f)

	accel_xout_scaled = accel_xout / SCALEa
	accel_yout_scaled = accel_yout / SCALEa
	accel_zout_scaled = accel_zout / SCALEa	

	print "accel_xout: ", accel_xout, " scaled: %.3f" %accel_xout_scaled
	print "accel_yout: ", accel_yout, " scaled: %.3f" %accel_yout_scaled
	print "accel_zout: ", accel_zout, " scaled: %.3f" %accel_zout_scaled

	print "------------"
	print "gyroscope data, cnt =%d" %i
	gyro_xout = read_word_2c(0x43)
	gyro_yout = read_word_2c(0x45)
	gyro_zout = read_word_2c(0x47)

	gyro_xout_scaled = gyro_xout / SCALEg
	gyro_yout_scaled = gyro_yout / SCALEg
	gyro_zout_scaled = gyro_zout / SCALEg	

	print "gyro_xout: ", gyro_xout, " scaled: %.3f" %gyro_xout_scaled
	print "gyro_yout: ", gyro_yout, " scaled: %.3f" %gyro_yout_scaled
	print "gyro_zout: ", gyro_zout, " scaled: %.3f" %gyro_zout_scaled

	mag_zout = read_word_B(0x07) 

	print("mag_zout", mag_zout)
	time.sleep(0.5)