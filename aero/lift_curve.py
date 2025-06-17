import numpy as np
import matplotlib.pyplot as plt
import sys
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType, ISA, AircraftType
from aero.lateral_centre import lateral_centre as la



class lift_curve():
    def __init__(self):
        self.aircraft_data = Data("design3.json")

        self.AR=self.aircraft_data.data['outputs']['wing_design']['aspect_ratio']
        self.taper=self.aircraft_data.data['outputs']['wing_design']['taper_ratio']
        self.cD_0=self.aircraft_data.data['inputs']['Cd0']
        self.e=self.aircraft_data.data['inputs']['oswald_factor']
        self.sweep_c4=self.aircraft_data.data['outputs']['wing_design']['sweep_c_4']
        self.b=self.aircraft_data.data['outputs']['wing_design']['b']
        self.lh=self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']
        self.zh=self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height']
        
        #importing lift curve data which is generated from xfoil without WIG
        self.data=np.loadtxt('aero\\lift curve no WIG.txt')
        self.data_span = np.loadtxt('aero\\spanwise curves.txt')
        self.data_tail_NACA0012 = np.loadtxt('aero\\Tail_NACA0012_XFLR.txt')
        #print(self.data)

        #angle of attack data
        self.alpha=self.data[:, 0]
        self.alpha_tail = self.data_tail_NACA0012[:,0]

        #lift coefficient data
        self.cl_lst=self.data[:,1] / (1+ self.data[:,1]/(np.pi * self.AR))
        # print(max(self.cl_lst))
        self.cd_lst=self.data[:,2]
        self.cm_lst=self.data[:,3]
   
        self.cl_lst_tail = self.data_tail_NACA0012[:,1]

        self.y_spanwise = self.data_span[:0]
        self.chord_spanwise = self.data_span[:1]
        self.Cl_spanwise = self.data_span[:2]
        self.iCd_spanwise = self.data_span[:3]



    
    def interpolate_Cl(self, alpha,h_b='no'):   #gets a cl for a given alpha and linearly interpolates
        #distance list in alpha
        # print(self.alpha)
        # print(alpha)
        dist=abs(self.alpha-alpha)
        # print(dist)


        #determines the index of the closest alpha
        ind=np.where(min(dist)==dist)[0][0]
        # print(ind)

        #uses point below
        if h_b=='no':
            if dist[ind] > 0:
                cl=self.cl_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cl_lst[ind]-self.cl_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])

            #uses point ahead
            elif dist[ind]<0:
                cl=self.cl_lst[ind]+(alpha-self.alpha[ind])*(self.cl_lst[ind+1]-self.cl_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])

            #uses selected alpha
            elif dist[ind]==0:
                cl=self.cl_lst[ind]


        else: 
            if dist[ind] > 0:
                cl=self.Cl_correction_GE(h_b,cl=self.cl_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cl_lst[ind]-self.cl_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1]))

            #uses point ahead
            elif dist[ind]<0:
                cl=self.Cl_correction_GE(h_b,cl=self.cl_lst[ind]+(alpha-self.alpha[ind])*(self.cl_lst[ind+1]-self.cl_lst[ind])/(self.alpha[ind+1]-self.alpha[ind]))

            #uses selected alpha
            elif dist[ind]==0:
                cl=self.Cl_correction_GE(h_b,cl=self.cl_lst[ind])

        #returns cl
        return cl
    
    def Cl_correction_GE(self,h_b,cl='no'):
        if cl=='no':
            Cl_arr = self.cl_lst
        else: 
            Cl_arr=cl
        delta_L = 1 - 2.25 * (self.taper**0.00273 - 0.997)*(self.AR*0.717 + 13.6)
        Cl_arr_GE = (1 + delta_L * (288*(h_b)**0.787 * np.exp(-9.14*h_b**0.327))/(self.AR*0.882)) * Cl_arr
        return Cl_arr_GE



    
    # def interpolate_Cd(self, alpha):   #gets a cd for a given alpha and linearly interpolates
    #     #distance list in alpha
    #     # print(self.alpha)
    #     # print(alpha)
    #     dist=abs(self.alpha-alpha)
    #     # print(dist)


    #     #determines the index of the closest alpha
    #     ind=np.where(min(dist)==dist)[0][0]
    #     # print(ind)

    #     #uses point below
    #     if dist[ind] > 0:
    #         cd=self.cd_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cd_lst[ind]-self.cd_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        
    #     #uses point ahead
    #     elif dist[ind]<0:
    #         cd=self.cd_lst[ind]+(alpha-self.alpha[ind])*(self.cd_lst[ind+1]-self.cd_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        
    #     #uses selected alpha
    #     elif dist[ind]==0:
    #         cd=self.cd_lst[ind]

    #     #returns cl
    #     return cd
 
    def interpolate_Cm(self, alpha):   #gets a cl for a given alpha and linearly interpolates
        #distance list in alpha
        # print(self.alpha)
        # print(alpha)
        dist=abs(self.alpha-alpha)
        # print(dist)


        #determines the index of the closest alpha
        ind=np.where(min(dist)==dist)[0][0]
        # print(ind)

        #uses point below
        if dist[ind] > 0:
            cm=self.cm_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cm_lst[ind]-self.cm_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        
        #uses point ahead
        elif dist[ind]<0:
            cm=self.cm_lst[ind]+(alpha-self.alpha[ind])*(self.cm_lst[ind+1]-self.cm_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        
        #uses selected alpha
        elif dist[ind]==0:
            cm=self.cm_lst[ind]

        #returns cl
        return cm
    
    def calc_drag(self,h_b='no',alpha='n',cl='n'):   #calculates the drag for only the wing using method from paper the values still need to be updated
        
        #correction factor
        if h_b =='no':
            sigma=0
        else:
            sigma=np.exp(-2.48*(h_b)**(0.768))

        # calculates cl for given alpha
        if cl=='n':
            cl=self.interpolate_Cl(alpha,h_b)

        #calculates induced drag with correction
        CD_i=cl**2/(np.pi*self.AR*self.e)*(1-sigma)

        #adds up drag
        CD=self.cD_0+CD_i

        #returns drag coeff
        return CD
    
    def calc_drag_butbetter(self,h_b='no',alpha='n',cl='n'):   #calculates the drag for only the wing using method from paper the values still need to be updated
        
        delta_D = 1 - 0.157 * (self.taper**0.775 - 0.373)*(self.AR**0.417)
        #correction factor
        if h_b=='no':
            sigma = 0
        else:
            sigma=delta_D*np.exp(-4.74*(h_b)**0.814) - h_b**2*np.exp(-3.88*(h_b)**0.758)

        # calculates cl for given alpha
        if cl=='n':
            if h_b=='no':
                cl=self.interpolate_Cl(alpha)
            else:
                cl=self.interpolate_Cl(alpha,h_b=h_b)

        #calculates induced drag with correction
        CD_i=cl**2/(np.pi*self.AR*self.e)*(1-sigma)

        #adds up drag
        CD=self.cD_0+CD_i

        #returns drag coeff
        return CD
    
    def lift_dist(self,V,ct,cr,rho=1.225,alpha='n',cl='n'):  #makes lift distrubution for now this looks like a tent is not to be trusted
        
        #makes a function of the chord of the airplane
        def chord(y,ct,cr):
            return 2*(ct-cr)*abs(y)+cr
        
        #makes spanwise list
        self.ylst=np.arange(-0.5,0.5,0.01)

        #makes list of the chord at spanwise location
        chord_lst=chord(self.ylst,ct,cr)

        #calculates cl for certain alpha
        if cl=='n':
            cl=self.interpolate(alpha)

        #makes spanwise lift distr
        self.L_lst=0.5*rho*V**2*chord_lst*cl


    def calc_moment_ac(self,x__c=0.25): #calculates the cmac and xac
        #general parameters
        self.x__c=x__c
        self.cmx__c=np.array([])   #cm at x__C
        self.xac=np.array([])


        self.seg1=[-4.5,3]   #alpha
        self.cmac_seg1=np.array([])
        self.alpha_seg1=np.array([])
        self.xac_seg1=np.array([])
        
        self.seg2=[6,11]  #alpha
        self.cmac_seg2=np.array([])
        self.alpha_seg2=np.array([])
        self.xac_seg2=np.array([])

        for i in range(len(self.cm_lst)):

            #calculating cmx__c
            cmx__c=self.cl_lst[i]*(0.25-self.x__c)+self.cm_lst[i]
            self.cmx__c=np.append(self.cmx__c,cmx__c)

            

            #calculates xac on a certain point
            if i==len(self.cm_lst)-1: #if last angle of attack
                xac=(0-self.cm_lst[i])/(0-self.cl_lst[i])+0.25
                self.xac=np.append(xac,self.xac)

            
            else:  #if normal
                if (self.cl_lst[i+1]-self.cl_lst[i])!=0:
                    xac=(self.cm_lst[i+1]-self.cm_lst[i])/(self.cl_lst[i+1]-self.cl_lst[i])+0.25
                else: #replace by 1/4
                    xac=0.25
                self.xac=np.append(self.xac,xac)


            #extracts xacs for segments
            if self.alpha[i]>=self.seg1[0] and self.alpha[i]<=self.seg1[1]:
                self.xac_seg1=np.append(self.xac_seg1,xac)

            if self.alpha[i]>=self.seg2[0] and self.alpha[i]<=self.seg2[1]:
                self.xac_seg2=np.append(self.xac_seg2,xac)

        #calculates xac for certain segment
        self.xac_segm1=np.mean(self.xac_seg1)
        self.xac_segm2=np.mean(self.xac_seg2)


        #calc cmac for segments
        for i in range(len(self.cm_lst)):
            if self.alpha[i]<=self.seg1[1] and self.alpha[i]>=self.seg1[0]:
                cmac=self.cl_lst[i]*(0.25-self.xac_segm1)+self.cm_lst[i]
                self.cmac_seg1=np.append(self.cmac_seg1,cmac)
                self.alpha_seg1=np.append(self.alpha_seg1,self.alpha[i])

            if self.alpha[i]<=self.seg2[1] and self.alpha[i]>=self.seg2[0]:
                cmac=self.cl_lst[i]*(0.25-self.xac_segm2)+self.cm_lst[i]
                self.cmac_seg2=np.append(self.cmac_seg2,cmac)
                self.alpha_seg2=np.append(self.alpha_seg2,self.alpha[i])
        
        self.cmac_segm1=np.mean(self.cmac_seg1)
        self.cmac_segm2=np.mean(self.cmac_seg2)
        
        #calc region in between
        self.segm15=np.arange(self.seg1[1],self.seg2[0],0.01)
        self.xac_segm15=self.xac_segm1-(self.xac_segm2-self.xac_segm1)/(self.seg1[1]-self.seg2[0])*(self.segm15-self.seg1[1])



        #print
        print('--------cmac-------')
        print(f'cmac for {self.seg1[0]} < α < {self.seg1[1]} = {round(self.cmac_segm1,4)} and xac = {round(self.xac_segm1,3)}.')
        print(f'cmac for {self.seg2[0]} < α < {self.seg2[1]} = {round(self.cmac_segm2,4)} and xac = {round(self.xac_segm2,3)}.')
        print('\t')
    

    def plot_moment_ac(self): #plots xac and mx/c

        #calc cmac and xac
        self.calc_moment_ac()

        fig=plt.figure()
        ax=fig.subplots(2,2)

        #cmx__c plot
        ax[0][0].plot(self.alpha,self.cmx__c)
        # ax[0][0].set_title('cm at x/c for alpha')
        ax[0][0].set_ylim(0,-0.2)
        ax[0][0].set_ylabel('cm')
        ax[0][0].set_xlabel('α (degrees)')
        


        #x_ac plot
        ax[0][1].plot(self.alpha,self.xac)
        # ax[0][1].set_title('Position of the ac at alpha')
        ax[0][1].set_ylim(-1,1)
        ax[0][1].set_ylabel('x_ac')
        ax[0][1].set_xlabel('α (degrees)')
        ax[0][1].hlines(self.xac_segm1,self.seg1[0],self.seg1[1],color='red',linestyle='dashed')
        ax[0][1].hlines(self.xac_segm2,self.seg2[0],self.seg2[1],color='red',linestyle='dashed')
        ax[0][1].plot(self.segm15,self.xac_segm15,color='red',linestyle='dashed')

        #cmac1
        ax[1][0].plot(self.alpha_seg1,self.cmac_seg1)
        # ax[1][0].set_title('cm at first range of alpha')
        ax[1][0].set_ylim(-0.09,-0.1)
        ax[1][0].set_xlim(-7,22)
        ax[1][0].set_ylabel('cm')
        ax[1][0].set_xlabel('α (degrees)')

        #cmac2
        ax[1][1].plot(self.alpha_seg2,self.cmac_seg2)
        ax[1][1].set_ylim(-0.25,-0.27)
        ax[1][1].set_xlim(-7,22)
        # ax[1][1].set_title('cm at second range of alpha')
        ax[1][1].set_ylabel('cm')
        ax[1][1].set_xlabel('α (degrees)')

        
        plt.show()
            
    def dcl_dalpha(self,h__b='no'):
    # Convert alpha to numpy array if it's not already
        alpha_arr = np.array(self.alpha)
        if h__b=='no':
            cl_arr = np.array(self.cl_lst)
            # print((cl_arr[0]-cl_arr[100])/(alpha_arr[0]-alpha_arr[100]))
        else:
            cl_arr=self.Cl_correction_GE(h_b=h__b)
            # print((cl_arr[0]-cl_arr[100])/(alpha_arr[0]-alpha_arr[100]))
        

        # Find index where alpha is closest to 5
        
        idx=np.argmin(np.abs(alpha_arr - 7))
        # print(alpha_arr[idx])
        
        # Slice arrays up to and including that index
        alpha_fit = alpha_arr[:idx+1]
        cl_fit = cl_arr[:idx+1]
        
        # Perform linear fit
        self.slope, intercept = np.polyfit(alpha_fit, cl_fit, 1)

        return self.slope, intercept

    def dcl_dalpha_tail(self):
        # NACA0012_data = self.cl_lst_tail
        # self.slope_tail = self.dcl_dalpha(NACA0012_data)
        alpha=self.data_tail_NACA0012[:,0]
        cl=self.data_tail_NACA0012[:,1]
        cd=self.data_tail_NACA0012[:,2]

        idx = np.argmin(np.abs(alpha - 7))

        # Slice arrays up to and including that index
        alpha_fit = alpha[:idx+1]
        cl_fit = cl[:idx+1]
        
        # Perform linear fit
        self.slope_tail, self.intercept_tail = np.polyfit(alpha_fit, cl_fit, 1)

        # print(self.slope_tail)
        return self.slope_tail

    def make_plot_data(self):
        #drag polar
        self.ind_lst=[]
        self.ind_lst2 = []
        self.ind_lst3 = []
        self.ind_lst4 = []

        #iteration over all alpha
        for i in range(len(self.alpha)):
            #calculation of drag 
            # self.ind_lst.append(self.calc_drag(h_b=0.050,alpha=self.alpha[i]))
            # self.ind_lst2.append(self.calc_drag(h_b='no',alpha=self.alpha[i]))

            self.ind_lst3.append(self.calc_drag_butbetter(h_b=0.050,alpha=self.alpha[i]))
            self.ind_lst4.append(self.calc_drag_butbetter(h_b='no',alpha=self.alpha[i]))
        

        #cl curve
        par0,par1=self.dcl_dalpha()
        self.cl_fit_no_WIG=par0*self.alpha+par1
        self.cl_lst_GE = self.Cl_correction_GE(h_b=0.05)

        #L/D
        self.L_D_lst=self.cl_lst/self.ind_lst4
        self.L_D_GE_lst=self.cl_lst_GE/self.ind_lst3

        


    def plotting(self):
        

        fig=plt.figure()
        ax=fig.subplots(2,2)

        # drag polar
        # ax[0][0].set_title('Drag Polar')
        
        # ax[0][0].plot(self.alpha[0:self.idx_cl_max], self.ind_lst[:self.idx_cl_max], label='h_b=0.050(old)') # Add a label
        # ax[0][0].plot(self.alpha[0:self.idx_cl_max], self.ind_lst2[:self.idx_cl_max], label='out GE(old)') # Add a label
        ax[0][0].plot(self.cl_lst_GE[:self.idx_cl_max], self.ind_lst3[:self.idx_cl_max], label='h_b=0.050') # Add a label
        ax[0][0].plot(self.cl_lst[:self.idx_cl_max], self.ind_lst4[:self.idx_cl_max], label='out GE') # Add a label
        ax[0][0].set_xlabel('Cl')
        # ax[0][0].set_ylabel('Cd')
        ax[0][0].legend() # Call legend() to display the labels


        #lift curve
        # ax[0][1].set_title('lift curve')
        ax[0][1].set_xlabel('α (degrees)')
        ax[0][1].set_ylabel('Cl')
        ax[0][1].plot(self.alpha[:self.idx_cl_max],self.cl_lst[:self.idx_cl_max], label='No GE')
        ax[0][1].plot(self.alpha[:self.idx_cl_max],self.cl_lst_GE[:self.idx_cl_max], label='GE')
        # ax[0][1].plot(self.alpha[:self.idx_cl_max],self.cl_fit_no_WIG[:self.idx_cl_max], '--r', label='no GE fit')
        ax[0][1].legend()

        #L/D
        # ax[1][0].set_title('L/D')
        ax[1][0].set_xlabel('α (degrees)')
        ax[1][0].set_ylabel('L/D')
        ax[1][0].set_ylim(0,70)
        ax[1][0].plot(self.alpha[:self.idx_cl_max],self.L_D_lst[:self.idx_cl_max], label='No GE')

        #L/D WIG
        ax[1][0].plot(self.alpha[:self.idx_cl_max],self.L_D_GE_lst[:self.idx_cl_max], label='GE')
        ax[1][0].legend()


        #Spanwise distribution
        # ax[1][1].set_title('lift spanwise')
        ax[1][1].plot(self.span, self.Cl_array_span)
        ax[1][1].set_ylim(0,2)
        ax[1][1].set_xlim(-40,40)
        ax[1][1].vlines([min(self.span),max(self.span)],0,self.Cl_array_span[0])

        plt.show()

    def printing(self):
        self.make_plot_data()
        print('---------clmax----------')
        print(f'Cl_max_GE: {round(max(self.cl_lst_GE),3)}')
        for i, cl in enumerate(self.cl_lst_GE):
            if cl == max(self.cl_lst_GE):
                print(f'alpha for Clmax: {self.alpha[i]}')
                self.cL_max=cl
                self.idx_cl_max=i
                break
        print('\t')

        print('--------L/D---------')
        print(f'L/D_max_GE: {round(max(self.L_D_GE_lst),1)}')
        for i, cl in enumerate(self.L_D_GE_lst):
            if cl == max(self.L_D_GE_lst):
                print(f'alpha for max L/D: {self.alpha[i]} degrees')
                
                break
        print('\t')

        #spanwise distr this is not prety but well coding
        CL = self.cL_max
        b=self.aircraft_data.data['outputs']['wing_design']['b']/2
        self.span = np.arange(-b, b, 0.01)
        self.Cl_array_span = CL * np.ones(np.shape(self.span))
    

    def calc_e(self):
        obj=la()
        obj.run()
        self.e_n=obj.e

    def downwash_gradient(self):
        M=0.3
        beta=np.sqrt(1-M**2)
        sweep50=np.arctan(np.tan(self.sweep_c4*np.pi/180)-4/self.AR*(0.5-0.25)*(1-self.taper)/(1+self.taper))
        k=1
        clM=2*np.pi*self.AR/(2+np.sqrt(self.AR**2*beta/k*(1+np.tan(sweep50)**2/beta**2)+4))
        cl0=2*np.pi*self.AR/(2+np.sqrt(self.AR**2/k*(1+np.tan(sweep50)**2)+4))
        clM__cl0= clM/cl0
        kA=1/self.AR-1/(1+self.AR**1.7)
        klam=(10-3*self.taper)/7
        kh=(1-abs(self.zh/self.b))/((2*self.lh/self.b)**(1/3))
        self.downwash_gr=4.44*(kA*klam*kh*np.sqrt(np.cos(self.sweep_c4*np.pi/180)))**(1.19)*clM__cl0

    def inp_json(self):
        self.slope,l=self.dcl_dalpha(0.05)
        aerodynamics = {
            'Cl_alpha': self.slope,
            'Cl_max': self.cL_max,
            't/c': 0.1426,
            'seg1_alpha': self.seg1,
            'cmac_seg1': self.cmac_segm1,
            'xac_seg1': self.xac_segm1,
            'seg2_alpha': self.seg2,
            'cmac_seg2': self.cmac_segm2,
            'xac_seg2': self.xac_segm2,
            'oswald_factor': self.e_n,
            'vertical/horizontal stab cl_alpha': self.slope_tail,
            'downwash_grad': self.downwash_gr}
        self.aircraft_data.data['outputs']['aerodynamics']=aerodynamics


        self.aircraft_data.save_design('design3.json')
        





if __name__ == "__main__":  #if run seperately  
    #defines instance
    
    curves=lift_curve()
    curves.plot_moment_ac()
    # print('jhi')
    # print(curves.dcl_dalpha())
    curves.printing()
    # print(curves.interpolate_Cl(5))
    curves.plotting()
    curves.calc_e()
    curves.dcl_dalpha_tail()
    curves.downwash_gradient()
    curves.inp_json()

    
    