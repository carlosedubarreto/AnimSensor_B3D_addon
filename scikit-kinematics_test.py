from skinematics import imus
import os
from sensors.xsens import XSens
from sensors.manual import MyOwnSensor
import numpy as np
myPath = 'C:\\Users\\Pichau\\AppData\\Local\\Programs\\Python\\Python37\\Lib\\site-packages\\skinematics\\tests'
def test_import_manual():
        # Get data, with a specified input from an XSens system
        in_file = os.path.join(myPath, 'data', 'data_xsens.txt')
        
        sensor = XSens(in_file=in_file, q_type=None)
        
        transfer_data = {'rate':sensor.rate,
                   'acc': sensor.acc,
                   'omega':sensor.omega,
                   'mag':sensor.mag}
        my_sensor = MyOwnSensor(in_file='My own 123 sensor.', in_data=transfer_data)
        
        print(my_sensor.rate)


def test_madgwick():
    
    from skinematics.sensors.manual import MyOwnSensor
    
    ## Get data
    imu = self.imu_signals
    in_data = {'rate' : imu['rate'],
        'acc' : imu['gia'],
        'omega' : imu['omega'],
        'mag' : imu['magnetic']}
    
    my_sensor = MyOwnSensor(in_file='Simulated sensor-data', in_data=in_data,
                            R_init = quat.convert(self.q_init, to='rotmat'),
                            pos_init = self.pos_init, 
                            q_type = 'madgwick')
    
    # and then check, if the quat_vector = [0, sin(45), 0]
    q_madgwick = my_sensor.quat
    
    result = quat.q_vector(q_madgwick[-1])
    correct = array([ 0.,  np.sin(np.deg2rad(45)),  0.])
    error = norm(result-correct)
    print(error)
    #self.assertAlmostEqual(error, 0)
    # self.assertTrue(error < 1e-3)
    
    ##inFile = os.path.join(myPath, 'data', 'data_xsens.txt')
    ##from skinematics.sensors.xsens import XSens
    
    ##initialPosition = array([0,0,0])
    ##R_initialOrientation = rotmat.R(0,90)
    
    ##sensor = XSens(in_file=inFile, R_init = R_initialOrientation, pos_init = initialPosition, q_type='madgwick')
    ##q = sensor.quat

def test_IMU_calc_orientation_position():
    """Currently, this only tests if the two functions are running through"""
    
    # Get data, with a specified input from an XSens system
    inFile = os.path.join(myPath, 'data', 'data_xsens.txt')
    initial_orientation = np.array([[1,0,0],
                                    [0,0,-1],
                                    [0,1,0]])
    initial_position = np.r_[0,0,0]

    from skinematics.sensors.xsens import XSens
    sensor = XSens(in_file=inFile, R_init=initial_orientation, pos_init=initial_position)
    sensor.calc_position()
    print('done')

test_import_manual()

test_IMU_calc_orientation_position()
