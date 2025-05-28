import numpy as np
import matplotlib.pyplot as plt




class lift_curve():
    def __init__(self):
        #importing lift curve data which is generated from xfoil without WIG
        self.data=np.loadtxt('aero\\lift curve WIG.txt')
        #print(self.data)

        #angle of attack data
        self.alpha=self.data[:, 0]

        #lift coefficient data
        self.cl_lst=self.data[:,1]
        self.cd_lst=self.data[:,2]


    
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
    
    def interpolate_Cd(self, alpha):   #gets a cl for a given alpha and linearly interpolates
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
            cl=self.cd_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cd_lst[ind]-self.cd_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        
        #uses point ahead
        elif dist[ind]<0:
            cl=self.cd_lst[ind]+(alpha-self.alpha[ind])*(self.cd_lst[ind+1]-self.cd_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        
        #uses selected alpha
        elif dist[ind]==0:
            cl=self.cd_lst[ind]

        #returns cl
        return cl
    
    def calc_drag(self,AR,e,h_b,alpha='n',cl='n',CD_0=0.0234):   #calculates the drag using method from paper
        
        #correction factor
        sigma=np.exp(-2.48*(h_b)**(0.768))

        # calculates cl for given alpha
        if cl=='n':
            cl=self.interpolate(alpha)

        #calculates induced drag with correction
        CD_i=cl**2/(np.pi*AR*e)*(1-sigma)

        #adds up drag
        CD=CD_0+CD_i

        #returns drag coeff
        return CD
    
    def lift_dist(self,V,ct,cr,rho=1.225,alpha='n',cl='n'):  #makes lift distrubution for now this looks like a tent is not to be trusted
        
        #makes a function of the chorsd of the airplane
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
    




            





if __name__ == "__main__":  #if run seperately
    #defines instance
    curves=lift_curve()

    #curves.interpolate(-4.2)

    #getting alpha data from instnace
    alphalst=curves.alpha
    ind_lst=[]

    #iteration over all alpha
    for i in range(len(alphalst)):
        #calculation of drag 
        ind_lst.append(curves.calc_drag(AR=8,e=0.85,h_b=0.050,alpha=alphalst[i]))
    
    fig=plt.figure()
    ax=fig.subplots(2,2)
    cl_lst=curves.cl_lst

    #drag polar
    ax[0][0].set_title('Drag polar')
    ax[0][0].plot(alphalst,ind_lst)
    
    #lift curve
    ax[0][1].set_title('lift curve')
    ax[0][1].plot(alphalst,cl_lst)

    #L/D
    L_D_lst=cl_lst/ind_lst
    ax[1][0].set_title('L/D')
    ax[1][0].set_ylim(0,70)
    ax[1][0].plot(alphalst,L_D_lst)

    #lift distribution
    #curves.lift_dist(alpha=2,V=90,ct=4.5,cr=11.5)
    #ax[1][1].set_title('spanwise lift distr')
    #ax[1][1].plot(curves.ylst,curves.L_lst)
    #ax[1][1].set_ylim(0,100000)

    #show
    plt.show()
    