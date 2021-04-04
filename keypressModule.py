# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 18:41:50 2021

@author: fgerm
"""

import pygame
import time 
def init():
    pygame.init()
    win=pygame.display.set_mode((100,100))
    
    #variables mm/s
    # speedx=20.0  
    # speedy=20.0
    # speedz=10.0
    # speeda=10.0         #this is in pulses per second
    # refresh_rate= 50.0  #times the function is called per second
    # deltax=speedx/refresh_rate
    # deltay=speedy/refresh_rate
    # deltaz=speedz/refresh_rate
    # deltaa=speeda/refresh_rate
    
def getKey(keyName):
    ans=False
    for eve in pygame.event.get():pass
    keyInput=pygame.key.get_pressed()
    mykey = getattr(pygame, 'K_{}'.format(keyName)) 
    if keyInput [mykey]:
        ans=True
    pygame.display.update()
    return ans

def readWASD():
    #return getKey('w')+getKey('a')*2+getKey('s')*4+getKey('d')*8+getKey('UP')*16+getKey('DOWN')*32+getKey('LEFT')*64+getKey('RIGHT')*128
    
    return [(getKey('w')-getKey('s')),(-getKey('a')+getKey('d')),(-getKey('UP')+getKey('DOWN')),(getKey('LEFT')-getKey('RIGHT'))]
    
def main():
    
    start = time.time() # gives current time in seconds since Jan 1, 1970 (in Unix)
    print(readWASD());
    while True:
        current_time = time.time()
        if current_time - start >= 1.0/50.0:
            break
            
if __name__ == '__main__':
    init()
    while True:
        main()