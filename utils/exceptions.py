from nelen_code.config.enums_dict import *
from nelen_code.config  import config 
import time

#timeouts
default_timeouts=25

def raise_except(axis,odrive_state_code):
    """
    Raises an exception if axis current state  is not odrive_state_code

    
    Parameters
    -----------
    axis: an odrive axis, ej: odrv0.axis0
    odrive_state_code: int state code from odrive.enums

    Returns
    ----------
    True if no exception was raised


    """
    current_state=axis.current_state
    if not (current_state==odrive_state_code):
        raise Exception('Failed trying to enter the ' + odrive_state[odrive_state_code]+
                        ' \n went to ' +odrive_state[current_state])
    else:
        return True

def timeout_except(axis, odrive_state_code,state_timeout=default_timeouts):
    """
    Raises an exception after state_timeout seconds if axis.state has not changed 

    
    Parameters
    -----------
    axis: an odrive axis, ej: odrv0.axis0
    odrive_state_code: int state code that the axis is currently in

    Returns
    ----------
    True if no exception was raised


    """

    retrys=0
    while axis.requested_state ==odrive_state_code:
        retrys +=1
        time.sleep(1)
        if retrys == state_timeout:
            raise Exception('timeout in ' + odrive_state[odrive_state_code])
    return True
