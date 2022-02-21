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




## Codo-muneca
# axis0
print('Working on z')
current_state_0 = odrvc.axis0.current_state

#only the encoder must be calibrated, the motor parameters are saved in the odrive
odrvc.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION  

while (current_state_0 == 1):
    odrvc.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(12)
    odrvc.axis0.requested_state = AXIS_STATE_HOMING
    time.sleep(15)
    current_state_0 = odrvc.axis0.current_state
    if current_state_0 == 1:
        print('Current axis successfully homed')
        odrvc.axis0.requested_state = 8
        current_state_0 = odrvc.axis0.current_state
        time.sleep(1)
        if current_state_0 == 8: 
            print('Current axis successfully enters control mod3')
    else:
        print(f'Control mode access failed. Current axis state = {current_state_0}')
        dump_errors(odrvc, True)
        time.sleep(1)

# axis1
print('Working on codo')
current_state_1 = odrvc.axis1.current_state
while (current_state_1 == 1):
    odrvc.axis1.requested_state = 7
    time.sleep(12)
    odrvc.axis1.requested_state = 11
    time.sleep(15)
    current_state_1 = odrvc.axis1.current_state
    if odrvc.axis1.current_state == 1:
        print('Current axis successfully homed')
        odrvc.axis1.requested_state = 8
        time.sleep(1)
        current_state_1 = odrvc.axis1.current_state
        if current_state_1 == 8:  
            print('Current axis successfully enters control mod3')
    else:
        print(f'Control mode access failed. Current axis state = {current_state_1}')
        dump_errors(odrvc, True)
        time.sleep(1)

## Hombro
print('Working on Hombro')
# axis 0
current_state_h = odrvh.axis0.current_state
while (current_state_h == 1):
    odrvh.axis0.requested_state = 7
    time.sleep(12)
    odrvh.axis0.requested_state = 11
    time.sleep(15) 
    current_state_h = odrvh.axis0.current_state
    if current_state_h == 1:
        print('Current axis successfully homed')
        odrvh.axis0.requested_state = 8
        time.sleep(1)
        current_state_h = odrvh.axis0.current_state
        if (current_state_h == 8):
            print('Current axis successfully enters control mod3')
            current_state_h = odrvh.axis0.current_state

    else:
        print(f'Homing failed. Current axis state = {current_state_h}')
        dump_errors(odrvh, True)
        time.sleep(1)

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
