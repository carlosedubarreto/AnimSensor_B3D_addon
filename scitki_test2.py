# C:/Users/Pichau/AppData/Local/Programs/Python/Python37/python.exe

import os
import numpy as np
# Set the in-file, initial sensor orientation
# in_file = r'tests/data/data_xsens.txt'
myPath = 'C:\\Users\\Pichau\\AppData\\Local\\Programs\\Python\\Python37\\Lib\\site-packages\\skinematics\\tests'
in_file = os.path.join(myPath, 'data', 'data_xsens.txt')
        
initial_orientation = np.array([[1,0,0],
                                [0,0,-1],
                                [0,1,0]])

# Choose a sensor
from skinematics.sensors.xsens import XSens
from skinematics.sensors.manual import MyOwnSensor

# Only read in the data
data = XSens(in_file, q_type=None)

# Read in and evaluate the data
sensor = XSens(in_file=in_file, R_init=initial_orientation)

# By default, the orientation quaternion gets automatically calculated, using the option "analytical"
q_analytical = sensor.quat

# Automatic re-calculation of orientation if "q_type" is changed
sensor.q_type = 'madgwick'
q_Madgwick = sensor.quat

sensor.q_type = 'kalman'
q_Kalman = sensor.quat

# Demonstrate how to fill up a sensor manually
in_data = {'rate':sensor.rate,
        'acc': sensor.acc,
        'omega':sensor.omega,
        'mag':sensor.mag}
my_sensor = MyOwnSensor(in_file='My own 123 sensor.', in_data=in_data)