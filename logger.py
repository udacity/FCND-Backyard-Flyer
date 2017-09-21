# -*- coding: utf-8 -*-
import numpy as np

class Logger:    
    def __init__(self,filename):
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
    
    
        
                
            
        

