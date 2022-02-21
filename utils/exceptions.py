from nelen_code.config.enums_dict import *
from nelen_code.config  import config 
import time

def raise_except(axis,odrive_state_code):
    current_state=axis.current_state
    if not (current_state==odrive_state_code):
        raise Exception('Failed trying to enter the ' + odrive_state[odrive_state_code]+
                        ' \n went to ' +odrive_state[current_state])
    else:
        return True

def timeout_except(axis, odrive_state_code):
    retrys=0
    while axis.requested_state ==odrive_state_code:
        retrys +=1
        time.sleep(1)
        if retrys == config.state_timeouts:
            raise Exception('timeout in ' + odrive_state[odrive_state_code])

