import numpy as np
import matplotlib.pyplot as plt
import math

red_ratio=7
r1=1
r2=0.5
maxtheta1=[0,math.pi]
martheta2=[math.pi/4,7*math.pi/4]
def directKin(theta1,theta2):
    theX=r1*math.cos(theta1)+r2*math.cos(theta2+theta1)
    theY=r1*math.sin(theta1)+r2*math.sin(theta2+theta1)
    return [theX,theY]


def inverseKin(x,y,ori="der"):
    rcuadrado=x**2+y**2
    B=(rcuadrado-r1**2-r2**2)/(2*r1*r2)
    theta2=math.atan2(math.sqrt(1-B*B),B)
    if (ori=="der"):
        theta1=math.atan2(y,x)-math.atan2(r2*math.sin(theta2),(r1+r2*math.cos(theta2)))
    elif (ori=="izq"):
        theta2=2*math.pi-theta2
        theta1=math.atan2(y,x)-math.atan2(r2*math.sin(theta2),(r1+r2*math.cos(theta2)))
    return [theta1,theta2]

def plotTheAngles(theta):
    x1=math.cos(theta[0])*r1
    y1=math.sin(theta[0])*r1
    endpoint=directKin(theta[0],theta[1])
    humerox=[0,x1]
    humeroy=[0,y1]
    rcx=[x1,endpoint[0]]
    rcy=[y1,endpoint[1]]
    plt.plot(humerox,humeroy,color="red")
    plt.plot(rcx,rcy)
    plt.xlim([-2,2])
    plt.ylim([-2,2])
    print("humero :")
    print(math.sqrt(x1**2+y1**2))
    print("RC: ")
    print(math.sqrt((rcx[0]-rcx[1])**2+(rcy[0]-rcy[1])**2))
    plt.show