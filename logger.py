# -*- coding: utf-8 -*-
import numpy as np
import os

class Logger:    
    def __init__(self,*args):
        if len(args)==0:
            filename = os.path.join("Logs","NavLog.txt")
        if len(args)==1:
            filename = os.path.join("Logs",args[0])
        elif len(args)==2:
            filename = os.path.join(args[0],args[1])
        
        self.log = open(filename,'w')
        self.num_data = 0
        self.open = True
        
    def close(self):
        self.log.close()
        self.open=False
    
    def log_data(self,data):
        if(self.num_data == 0):
            self.num_data = len(data)
            
        if(len(data)!=self.num_data):
            print("Logger: Invalid number of entries")
            return
        
        for i in range(len(data)):
            if type(data[i]) == float:
                self.log.write('{0:.7f}'.format(data[i]))
            else:
                self.log.write(data[i].__str__())
            if i != len(data)-1:
                self.log.write(',')
            
        self.log.write('\n')
        
    def log_telemetry_data(self,data):
        for i in range(len(data)):
            if type(data[i]) == float:
                self.log.write('{0:.7f}'.format(data[i]))
            else:
                self.log.write(data[i].__str__())
            if i != len(data)-1:
                self.log.write(',')
            
        self.log.write('\n')

def read_log(filename):
    """
    Returns a numpy 2D array of the data
    """
    return np.loadtxt(filename,delimiter=',',dtype='Float64')
    
    
        
                
            
        

