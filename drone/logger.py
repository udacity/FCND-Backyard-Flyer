# -*- coding: utf-8 -*-
import numpy as np
import os

class Logger:    
    def __init__(self,*args):
        if len(*args)==0:
            filename = os.path.join("Logs","NavLog.txt")
        if len(*args)==1:
            filename = os.path.join("Logs",*args[0])
        elif len(*args)==2:
            filename = os.path.path(*args[0],*args[1])
        
        self.log = open(filename,'w')
        self.num_data = 0
        
    def close(self):
        self.log.close()
    
    def log_data(self,data):
        if(self.num_data == 0):
            self.num_data = len(data)
            
        if(len(data)!=self.num_data):
            print("Logger: Invalid number of entries")
            return
        
        for i in range(len(data)):
            self.log.write('{0:.7f}'.format(data[i]))
            if i != len(data)-1:
                self.log.write(',')
            
        self.log.write('\n')

#Returns a numpy 2D array of the data
def read_log(filename):
    return np.loadtxt(filename,delimiter=',',dtype='Float64')
    
    
        
                
            
        

