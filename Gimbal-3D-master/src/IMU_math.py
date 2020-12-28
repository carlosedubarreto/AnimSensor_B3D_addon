"""
Library to handle all functionalities of the gyro
This includes reading and integrating the reading
"""
import numpy as np
from numpy.linalg import inv
import math
import smbus
import time

bus = None 	
address = None

def get_IMU_reading():
	"""
	Gets the IMU reading based on the circuit diagram
	IMU used is MPU 9255. 
	Please refer to: <link> for more details of our project

	The IMU will comminucate with the raspberry via address 0x68 (for our case)
	Based on data sheet, we write / read from the addresses:
	Accelerometer: 0x3b - 0x3e 
	Gyro: 0x43 - 0x48 

	Once we read the data, we have to scale it based on the proper value

	param: None
	rtype: np.array of reading from (gryo_x, gryo_y, gryo_z, accel_x, accel_y, accel_z)
	"""
	global bus
	global address
	
	power_mgmt_1 = 0x6b
	power_mgmt_2 = 0x6c
	bus = smbus.SMBus(1)            # I2C port 1
	address = 0x68 		        # This is the address value via the i2cdetect command or chip user guide
	SCALEa = 65536.0/4		# Scaling of accel and gyro readings 
	SCALEg = 131.0

	#Now wake the MPU 9255 up as it starts in sleep mode
	bus.write_byte_data(address, power_mgmt_1, 0)

	accel_xout = __read_word_2c(0x3b)
	accel_yout = __read_word_2c(0x3d)
	accel_zout = __read_word_2c(0x3f)

	accel_xout_scaled = accel_xout / SCALEa
	accel_yout_scaled = accel_yout / SCALEa
	accel_zout_scaled = accel_zout / SCALEa	

	gyro_xout = __read_word_2c(0x43)
	gyro_yout = __read_word_2c(0x45)
	gyro_zout = __read_word_2c(0x47)

	gyro_xout_scaled = gyro_xout / SCALEg
	gyro_yout_scaled = gyro_yout / SCALEg
	gyro_zout_scaled = gyro_zout / SCALEg	

	return np.array([gyro_xout_scaled, gyro_yout_scaled, gyro_zout_scaled, accel_xout_scaled, accel_yout_scaled, accel_zout_scaled])

def __read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def __read_word_2c(adr):
	val= __read_word(adr)
	if (val >= 0x8000):
		return - ((65535 - val) + 1)
	else:
		return val

def get_integrate_gyro(gyro_reading, smpl_time):
	"""
	Integrates the gyro reading given a sampling time.
	Integration is performed numerically

	param: 	gyro_reading -> numpy.ndarray type that is the reading of the angular velocities
			smpl_time -> float type that is the sampling rate of the IMU

	rtype:	numpy.ndarray type that is the result after performing integration 
	"""
	assert type(gyro_reading) is np.ndarray and type(smpl_time) is float
	return gyro_reading*smpl_time

def get_state_accel(accel_reading):
	"""
	Gets the state of the system based on accelerometer readings
	The formula used are:
		roll 	= atan(A_x / sqrt(A_y^2 + A_z^2))
		pitch 	= atan(A_y / sqrt(A_x^2 + A_z^2))
	** The limitation of the accelerometer is that it cannot calculate yaw rotations because that axis corresponds with gravity

	param: accel_reading -> numpy.nparray type that is the reading of the accelerometer

	rtype: numpy.nparray type that is represents the tile angles in (pitch roll)
	"""
	from math import atan
	from math import sqrt
	assert type(accel_reading) is np.ndarray
	A_x = accel_reading[0]; A_y = accel_reading[1]; A_z = accel_reading[2]

	roll 	= atan(A_x / sqrt(A_y**2 + A_z**2))
	pitch 	= atan(A_y / sqrt(A_x**2 + A_z**2))

	return np.array([pitch, roll, 0])

def kalman_filter(F, B, R, Q, u_k, drift, true_state_km1, z_k, P_km1_km1):
	"""
	Function to implement the Kalman kalman filter.
	Assumptions made are that the distribution of the system is gaussian with mean and variance
	All bias from accelerometer and gyro are assumed to be constant
	We first have to decompose the inputs which are 3D into 1D and process each input separately

	Theory for 1D Kalman filtering: 
	** We iterate our filter 3 times for pitch / roll / yaw calculations

	Assume that systems true state at time k given state at time k-1 is modelled as:
	>>> x_k_km1 = F*x_km1_km1 + B*u_k + w_k
	Where w_k is the process noise 

	The measured value of the system at time k relates to x_k_km1 by:
	>>> z_k = H*x_k_km1 + v_k
	v_k is the noise 

	Start by calculating x_k_km1 ignoring the noise present
	>>> x_k_km1 = F*x_km1_km1 + B*u_k

	Calculate the innovation (y_k) using
	>>> y_k = z_k - H*x_k_km1

	Calculate the error covariance matrix P_k_km1 using
	>>> P_k_km1 = F *P_km1_km1 *F' + Q

	Calculate the innovation covariance matrix using
	>>> S_k = H *P_k_km1 *H' + R

	Calculate the kalman gain using
	>>> K_k = P_k_km1 *H' *inv(S_k)

	Calculate the true state x_k_k using
	>>> x_k_k = x_k_km1 + K_k *y_k

	Calculate the error covariance matrix at time k 
	>>> P_k_k = (I - K_k *H) *P_k_km1

	param: 	F: State transition matrix 						- 2 x 2
		B: Mapping matrix for control input					- 2 x 1
		R: Measurement covariance matrix 					- 1 x 1			
		Q: Noise covariance matrix 						- 2 x 2
		vel: measured angular velocity at time k 				- 1 x 3
		drift: measured drift of gyro 						- 1 x 3
		true_state_km1: The true state of the system at time k-1		- 1 x 3
		z_k: The measured state of the system at time k 			- 1 x 3
		P_k_km1: The error covariance matrix at time k-1			- [2 x 2, 2 x 2, 2 x 2]

	rtype: 	true_state_k: The true state of the system at time k 			- 1 x 3
			P_k_k_all: The collective error covariance matrix at time k 	- [2 x 2, 2 x 2, 2 x 2]
	"""
	H               = np.matrix([1, 0])			# Maps true state to measured reading
	x_km1_km1 	= [np.transpose(np.matrix(i)) for i in zip(true_state_km1, drift)]
	true_state_k    = np.array([])
	P_k_k_all       = []

	for j in range(3):
                x_k_km1 	= F *x_km1_km1[j] + B *u_k[j]	
		y_k 		= z_k[j] - H *x_k_km1			
		P_k_km1 	= F *P_km1_km1[j] *np.transpose(F) + Q
		S_k 		= H *P_k_km1 *np.transpose(H) + R
		K_k 		= P_k_km1 *np.transpose(H) *inv(S_k)
		if j == 2:
                        # x_k_km1 = 0
                        K_k = 0
                        
		x_k_k 		= x_k_km1 + K_k *y_k
		P_k_k 		= (np.identity(2) - K_k *H) *P_k_km1
		
		true_state_k = np.append(true_state_k, x_k_k[0])		# Appends mean position
		P_k_k_all.append(P_k_k)						# Appends error covariance matrix
	return true_state_k, P_k_k_all

if __name__ == "__main__":
	# To test the Kalman filter
	F = np.matrix([[1,-0.1],[0,1]])
	B = np.matrix([[0.1],[0]])
	R = 0.01
	Q = np.matrix([[0.01,0],[0,0.03]])
	u_k = np.array([10,-10,-10])
	drift = np.array([0,0,0])
	true_state_km1 = np.array([0,1,2])
	z_k = np.array([0.1,0.9,1.9])
	P_km1_km1 = [np.asmatrix(np.zeros((2,2))) for i in range(3)]

	kalman_filter(F, B, R, Q, u_k, drift, true_state_km1, z_k, P_km1_km1)
