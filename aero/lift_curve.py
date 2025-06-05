import numpy as np
import matplotlib.pyplot as plt
import sys


class lift_curve():
    def __init__(self):
        #importing lift curve data which is generated from xfoil without WIG
        self.data=np.loadtxt('aero\\lift curve no WIG.txt')
        self.data_span = np.loadtxt('aero\\spanwise curves.txt')
        #print(self.data)

        #angle of attack data
        self.alpha=self.data[:, 0]

        #lift coefficient data
        self.cl_lst=self.data[:,1]
        self.cd_lst=self.data[:,2]
        self.cm_lst=self.data[:,3]

        self.y_spanwise = self.data_span[:0]
        self.chord_spanwise = self.data_span[:1]
        self.Cl_spanwise = self.data_span[:2]
        self.iCd_spanwise = self.data_span[:3]



    
    def interpolate_Cl(self, alpha):   #gets a cl for a given alpha and linearly interpolates
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
            cl=self.cl_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cl_lst[ind]-self.cl_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        
        #uses point ahead
        elif dist[ind]<0:
            cl=self.cl_lst[ind]+(alpha-self.alpha[ind])*(self.cl_lst[ind+1]-self.cl_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        
        #uses selected alpha
        elif dist[ind]==0:
            cl=self.cl_lst[ind]

        #returns cl
        return cl
    
    def Cl_correction_GE(self,h_b=0.050, AR=8, taper = 0.4):
        Cl_arr = self.cl_lst
        delta_L = 1 - 2.25 * (taper**0.00273 - 0.997)*(AR*0.717 + 13.6)
        Cl_arr_GE = (1 + delta_L * (288*(h_b)**0.787 * np.exp(-9.14*h_b**0.327))/(AR*0.882)) * Cl_arr
        return Cl_arr_GE



    
    def interpolate_Cd(self, alpha):   #gets a cd for a given alpha and linearly interpolates
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
            cd=self.cd_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cd_lst[ind]-self.cd_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        
        #uses point ahead
        elif dist[ind]<0:
            cd=self.cd_lst[ind]+(alpha-self.alpha[ind])*(self.cd_lst[ind+1]-self.cd_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        
        #uses selected alpha
        elif dist[ind]==0:
            cd=self.cd_lst[ind]

        #returns cl
        return cd
 
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
    
    def calc_drag(self,h_b='no',AR=8,e=0.85,alpha='n',cl='n',CD_0=0.0169):   #calculates the drag for only the wing using method from paper the values still need to be updated
        
        #correction factor
        if h_b =='no':
            sigma=0
        else:
            sigma=np.exp(-2.48*(h_b)**(0.768))

        # calculates cl for given alpha
        if cl=='n':
            cl=self.interpolate_Cl(alpha)

        #calculates induced drag with correction
        CD_i=cl**2/(np.pi*AR*e)*(1-sigma)

        #adds up drag
        CD=CD_0+CD_i

        #returns drag coeff
        return CD
    
    def calc_drag_butbetter(self,h_b='no',AR=8,e=0.85,alpha='n',cl='n',CD_0=0.0169):   #calculates the drag for only the wing using method from paper the values still need to be updated
        
        taper = 0.4
        delta_D = 1 - 0.157 * (taper**0.775 - 0.373)*(AR**0.417)
        #correction factor
        if h_b=='no':
            sigma = 0
        else:
            sigma=delta_D*np.exp(-4.74*(h_b)**0.814) - h_b**2*np.exp(-3.88*(h_b)**0.758)

        # calculates cl for given alpha
        if cl=='n':
            cl=self.interpolate_Cl(alpha)

        #calculates induced drag with correction
        CD_i=cl**2/(np.pi*AR*e)*(1-sigma)

        #adds up drag
        CD=CD_0+CD_i

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


    def calc_moment_ac(self):
        self.x__c=0.3
        self.cmx__c=self.cl_lst*(0.25-self.x__c)+self.cm_lst
        self.xac=np.array([])

        for i in range(len(self.cm_lst)):
            if i==len(self.cm_lst)-1:
                None
            else:
                xac=(self.cm_lst[i+1]-self.cm_lst[i])/(self.cl_lst[i+1]-self.cl_lst[i])+0.25
                self.xac=np.append(xac,self.xac)
    

    def plot_moment_ac(self):
        fig=plt.figure()
        ax=plt.add_subplots(2)
            
    def dcl_dalpha(self):
    # Convert alpha to numpy array if it's not already
        alpha_arr = np.array(self.alpha)
        cl_arr = np.array(self.cl_lst)

        # Find index where alpha is closest to 5
        idx = np.argmin(np.abs(alpha_arr - 3))
        
        # Slice arrays up to and including that index
        alpha_fit = alpha_arr[:idx+1]
        cl_fit = cl_arr[:idx+1]
        
        # Perform linear fit
        slope, intercept = np.polyfit(alpha_fit, cl_fit, 1)

        # Plot original data and linear fit
        # plt.plot(alpha_arr, cl_arr, label='Lift Curve')
        # plt.plot(alpha_fit, slope * alpha_fit + intercept, label='Linear Fit', linestyle='--')
        # plt.xlabel("Angle of Attack (alpha)")
        # plt.ylabel("Lift Coefficient (Cl)")
        # plt.legend()
        # plt.grid(True)
        # plt.tight_layout()
        # plt.show()

        return slope




            





if __name__ == "__main__":  #if run seperately  
    #defines instance
    curves=lift_curve()

    #curves.interpolate(-4.2)

    #getting alpha data from instnace
    alphalst=curves.alpha
    ind_lst=[]
    ind_lst2 = []
    ind_lst3 = []
    ind_lst4 = []

    #iteration over all alpha
    for i in range(len(alphalst)):
        #calculation of drag 
        ind_lst.append(curves.calc_drag(AR=8,e=0.85,h_b=0.050,alpha=alphalst[i]))
        ind_lst2.append(curves.calc_drag(AR=8,e=0.85,h_b='no',alpha=alphalst[i]))

        ind_lst3.append(curves.calc_drag_butbetter(AR=8,e=0.85,h_b=0.050,alpha=alphalst[i]))
        ind_lst4.append(curves.calc_drag_butbetter(AR=8,e=0.85,h_b='no',alpha=alphalst[i]))

    
    fig=plt.figure()
    ax=fig.subplots(2,2)
    cl_lst=curves.cl_lst
    cl_lst_GE = curves.Cl_correction_GE()
    print(f'Cl_max_GE: {max(cl_lst_GE)}')
    for i, cl in enumerate(cl_lst_GE):
        if cl == max(cl_lst_GE):
            print(f'alpha for Clmax: {alphalst[i]}')
            break

                    

    # #drag polar
    # ax[0][0].set_title('Drag polar')
    # ax[0][0].plot(alphalst,ind_lst)
    # ax[0][0].plot(alphalst,ind_lst2)
    # ax[0][0].plot(alphalst, ind_lst3)
    # ax[0][0].plot(alphalst, ind_lst4)

    # drag polar
    ax[0][0].set_title('Drag polar')
    ax[0][0].plot(alphalst, ind_lst, label='h_b=0.050(old)') # Add a label
    ax[0][0].plot(alphalst, ind_lst2, label='out GE(old)') # Add a label
    ax[0][0].plot(alphalst, ind_lst3, label='h_b=0.050') # Add a label
    ax[0][0].plot(alphalst, ind_lst4, label='out GE') # Add a label
    ax[0][0].legend() # Call legend() to display the labels

    
    #lift curve
    ax[0][1].set_title('lift curve')
    ax[0][1].plot(alphalst,cl_lst, label='no GE')
    ax[0][1].plot(alphalst,cl_lst_GE, label='GE')
    ax[0][1].legend()

    #L/D
    L_D_lst=cl_lst/ind_lst
    ax[1][0].set_title('L/D')
    ax[1][0].set_ylim(0,70)
    ax[1][0].plot(alphalst,L_D_lst, label='old')

    #L/D WIG
    L_D_GE_lst=cl_lst_GE/ind_lst3
    ax[1][0].plot(alphalst,L_D_GE_lst, label='new')
    ax[1][0].legend()

    print(f'L/D_max_GE: {max(L_D_GE_lst)}')
    for i, cl in enumerate(L_D_GE_lst):
        if cl == max(L_D_GE_lst):
            print(f'alpha for max L/D: {alphalst[i]} degrees')
            break

    #Spanwise distribution
    CL = 1.8
    span = np.arange(-35, 35, 0.01)
    Cl_array_span = CL * np.ones(np.shape(span))
    ax[0][1].set_title('lift spanwise')
    ax[1][1].plot(span, Cl_array_span)


    

    #lift distribution
    #curves.lift_dist(alpha=2,V=90,ct=4.5,cr=11.5)
    #ax[1][1].set_title('spanwise lift distr')
    #ax[1][1].plot(curves.ylst,curves.L_lst)
    #ax[1][1].set_ylim(0,100000)

    #show
    plt.show()
    