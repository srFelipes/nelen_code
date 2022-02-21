#!/home/rosconcoco/nelen/bin/python

import time

import odrive
from odrive.enums import * 
from odrive.utils import *

from nelen_code.config import config
from nelen_code.utils import exceptions
odrvc = odrive.find_any(serial_number=config.rc_serial) # codo and z
odrvh = odrive.find_any(serial_number=config.hombro_serial) # hombro

print('Dumping errors codo-muneca')
dump_errors(odrvc, True)
print('Dumping errors hombro')
dump_errors(odrvh, True)

# TODO: Generalize calibration and close_loop_control access



def calibrate(axis):
    #only the encoder must be calibrated, the motor parameters are saved in the odrive
    #this should check if the axis changed correctly
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION  
    exceptions.odrive_state(axis,AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
    exceptions.timeout_except(axis,AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
    print('succesfully entered AXIS_STATE_ENCODER_OFFSET_CALIBRATION')  
    
    #execute homing
    axis.requested_state = AXIS_STATE_HOMING 
    exceptions.raise_except(axis,AXIS_STATE_HOMING)
    exceptions.timeout_except(axis,AXIS_STATE_HOMING)
    print('Current axis successfully homed')
    
    #execute control loop
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    exceptions.raise_except(axis, AXIS_STATE_CLOSED_LOOP_CONTROL)
    print('Current axis successfully enters control mod3')


# straight
# z
if should_comeback:
    offset_z_stg = -3.25 # 6.25
    odrvc.axis0.controller.move_incremental(offset_z_stg, False)
    time.sleep(5)
    # codo
    offset_c_stg = -2.6
    odrvc.axis1.controller.move_incremental(offset_c_stg, False)
    time.sleep(5)
    # hombro
    offset_h_stg = -1.75 
    odrvh.axis0.controller.move_incremental(offset_h_stg, False)
    time.sleep(5)
print("fin")

if __name__=='__init__':
    ## Hombro
    print('Working on Hombro')
    calibrate(odrvh.axis0)
    print('hombro calibrated')
    
    # odrv radio cubito, axis 0->Z, 1->codo
    
    #Z
    print('Working on z')
    calibrate(odrvc.axis0)
    print('z calibrated')
    
    #codo
    print('Working on codo')
    calibrate(odrvc.axis1)
    print('hombro calibrated')

