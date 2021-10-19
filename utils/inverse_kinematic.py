import numpy as np

def ik(x, y, L1, L2, orientation = 'izq', verbose = False):
    '''Calculate inverse kinematic of the arm:
    input:  
    x = x target position
    y = y target position
    L1 = humero lenght
    L2 = radio-cubito lenght
    output: 
    theta1 (hombro degree) theta2 (codo degree)'''
    max_theta_1=90.0/180.0*np.pi
    max_theta_2=135.0/180.0*np.pi
    #in_arccos = min(max((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2),-1) ,1)
    print("x = ") 
    print(x)
    print("y = ") 
    print(y)
    print("L1 = ") 
    print(L1)
    print("L2 = ") 
    print(L2)
         
    D =min(max((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2),-1),1)
    print(D)
            
    if orientation == 'izq':
        #theta_2 = np.arccos(in_arccos)
        theta_2 = np.arctan(np.sqrt(1-D**2)/D)
        if x==0:
            theta_1 = np.sign(y)*np.pi/2.0 - np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
        else:
            theta_1 = np.arctan(y/x) - np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
    elif orientation == 'der':
        #theta_2 = -np.arccos(in_arccos)
        theta_2 = np.arctan(-np.sqrt(1-D**2)/D)
        if x==0:
            theta_1 = np.sign(y)*np.pi/2.0 + np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
        else:
            theta_1 = np.arctan(y/x) + np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
    else:
        print('Orientation mode error. It should be equal to: str(der) or str(izq)')
    if verbose == True:
        print(f'Targets: x = {x}, y = {y}')
        print(f'Theta_1 = {theta_1}, Theta_2 = {theta_2}')

    if ( (np.abs(theta_1)<max_theta_1) and (np.abs(theta_2)<max_theta_2) ):
        print('das gut shieeet');
        return theta_1, theta_2
    elif (np.abs(theta_1)>max_theta_1):
        print('te passaste en theta1')
        print(theta_1*180.0/np.pi)
    else:
        print('te passaste en theta2')
        print(theta_2*180.0/np.pi)
