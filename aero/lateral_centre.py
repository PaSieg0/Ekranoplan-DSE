import numpy as np
import matplotlib.pyplot as plt
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType, ISA, AircraftType
import scipy.integrate as sc

from scipy.optimize import curve_fit

def f(x,a,b,c,d,e,f,g):
    return a*np.sqrt(b**2-x**2) +c*d*e*f*g    

class lateral_centre():
    def __init__(self):
        self.aircraft_data = Data("design3.json")
        self.import_data()
        

    def import_data(self):
        #importing data
        self.data=np.loadtxt("aero\\wing distr.txt")

        self.cd0=self.aircraft_data.data['inputs']['Cd0']

        #using data
        self.pos=self.data[:,0]
        self.n=int(self.pos.shape[0])*2
        self.cl=self.data[:,3]
        self.cm1__4=self.data[:,7]
        self.Cdi=self.data[:,5]

        #calc cl at 0
        cl0=-self.pos[0]*(self.cl[1]-self.cl[0])/(self.pos[1]-self.pos[0])+self.cl[0]
        cdi0=-self.pos[0]*(self.Cdi[1]-self.Cdi[0])/(self.pos[1]-self.pos[0])+self.Cdi[0]
        cm1__40=-self.pos[0]*(self.cm1__4[1]-self.cm1__4[0])/(self.pos[1]-self.pos[0])+self.cm1__4[0]
        #adding 0
        self.pos=np.append(0,self.pos)
        self.cl=np.append(cl0,self.cl)
        self.Cdi=np.append(cdi0,self.Cdi)
        self.cm1__4=np.append(cm1__40,self.cm1__4)

        #calc boundaries for intergration
        self.b=self.pos[-1]
        self.a=self.pos[0]
        
        #making y*cl
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

    
    def plot_distr(self):
        #importing pos and L
        pos,L,M,D=self.determine_distr(74.5/2,13.3,5.32,1.225,61.73*1.05)

        #making plots
        fig=plt.figure()
        ax=fig.subplots(2,1)
        ax[0].plot(pos,self.cll)
        # ax.vlines(self.mu*self.b,0,2.5,color='red',linestyle='dashed')
        ax[0].plot(self.pos,self.cl)
        ax[0].set_ylabel('Corrected cl')
        # ax.set_title('spanwise lift distribution')
        # ax.set_xlabel('spanwise position')
        # ax.set_ylabel('lift coefficient')
        # ax[1].set_xlim(-1.5,36.5)
        # ax[1].set_ylabel('L [N]')
        # ax[1].set_xlabel('y [m]')
        # ax[1].plot(pos,L)
        ax[1].plot(pos,M)
        # ax[2].set_xlim(-1.5,36.5)
        ax[1].set_ylabel('M [Nm]')
        ax[1].set_xlabel('y [m]')
        # ax[1].plot(pos,D)
        # # ax[3].set_xlim(-1.5,36.5)
        # ax[1].set_ylabel('D [N]')
        # ax[1].set_xlabel('y [m]')
        plt.show()


    def determine_distr(self,b__2,cr,ct,rho,v_inv,cl_max=2.2):
        #scaling position
        pos=self.pos*b__2/self.b

        #scalling cl
        C_L=sc.trapezoid(self.cl,self.pos)/self.b
        self.cll=self.cl*cl_max/C_L
        self.cm=self.cm1__4*cl_max/C_L
        self.cd=(self.Cdi+self.cd0)*(cl_max/C_L)**2

        #calculating l/q
        self.l__q=self.cl*((ct-cr)*pos/self.b+cr)
        l__q=self.cll*((ct-cr)*pos/b__2+cr)
        M__q=self.cm*((ct-cr)*pos/b__2+cr)**2
        D__q=self.cd*((ct-cr)*pos/b__2+cr)


        #calculating L
        L=l__q*0.5*rho*v_inv**2
        M=M__q*0.5*rho*v_inv**2
        D=D__q*0.5*rho*v_inv**2

        return pos,L,M,D




if __name__=='__main__':
    obj=lateral_centre()
    obj.run()
    obj.update_json()
    obj.plot_distr()
    