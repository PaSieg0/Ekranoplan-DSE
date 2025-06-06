import numpy as np
import matplotlib.pyplot as plt
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType, ISA, AircraftType
import scipy.integrate as sc

class lateral_centre():
    def __init__(self):
        self.import_data()
        self.aircraft_data = Data("design3.json")

    def import_data(self):
        self.data=np.loadtxt("aero\\wing distr.txt")
        # print(self.data)
        self.pos=self.data[:,0]
        #print(self.cl)
        # print(self.pos)
        self.n=int(self.pos.shape[0])*2
        #self.pos=self.data[int(self.n/2):self.n,0]
        #print(self.pos)
        self.cl=self.data[:,3]
        self.b=self.pos[-1]
        self.a=self.pos[0]
        self.cl_pos=self.pos*self.cl
        #print(self.cl_pos)
    
    def intergrate(self):
        I_2=sc.trapezoid(self.cl,self.pos)
        I_1=sc.trapezoid(self.cl_pos,self.pos)
        # print(I_2)
        
        I=I_1/I_2
        self.mu=I/max(self.pos)

    def oswald_calc(self):
        self.e=(4.5*(np.pi*self.mu)**2-12*np.pi*self.mu+9)**(-1)
        if __name__=='__main__':
            print('\t')
            print('----------e---------')
            print(f'e={round(self.e,3)}')

    def update_json(self):
        self.aircraft_data.data['inputs']['oswald_factor']=self.e
        self.aircraft_data.save_design('design3.json')

    def run(self):
        self.intergrate()
        self.oswald_calc()




if __name__=='__main__':
    obj=lateral_centre()
    #print(obj.intergrate())
    obj.run()
    obj.update_json()
    