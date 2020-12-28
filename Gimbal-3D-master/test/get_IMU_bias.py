"""
Simple test to show the bias of the IMU
"""

from main import gimbal_driver

if __name__ == "__main__":
	driver = gimbal_driver()
	bias = driver.get_IMU_bias()

	print ("Gyro bias in X: " + bias[0])
	print ("Gyro bias in Y: " + bias[1])
	print ("Gyro bias in Z: " + bias[2])
	print ("Accel bias in X: " + bias[3])
	print ("Accel bias in Y: " + bias[4])
	print ("Accel bias in Z: " + bias[5])
