import numpy as np
import matplotlib.pyplot as plt




class lift_curve():
    def __init__(self):
        self.data=np.loadtxt('aero\\lift curve.txt')
        #print(self.data)
        self.alpha=self.data[:, 0]
        self.cl_lst=self.data[:,1]

    def interpolate(self, alpha):
        dist=abs(self.alpha-alpha)
        #print(dist)
        mindist=min(dist)
        ind=np.where(mindist==dist)[0][0]
        #print(ind)
        if dist[ind] > 0:
            cl=self.cl_lst[ind-1]+(alpha-self.alpha[ind-1])*(self.cl_lst[ind]-self.cl_lst[ind-1])/(self.alpha[ind]-self.alpha[ind-1])
        elif dist[ind]<0:
            cl=self.cl_lst[ind]+(alpha-self.alpha[ind])*(self.cl_lst[ind+1]-self.cl_lst[ind])/(self.alpha[ind+1]-self.alpha[ind])
        elif dist[ind]==0:
            cl=self.cl_lst[ind]
        return cl
    
    def ind_drag(self,AR,e,h_b,alpha='n',cl='n'):
        sigma=np.exp(-2.48*(h_b)**(0.768))
        if cl=='n':
            cl=self.interpolate(alpha)
        CD_i=cl**2/(np.pi*AR*e)*(1-sigma)
        CD_0=0.0234
        CD=CD_0+CD_i
        return CD
    
    def lift_dist(self,V,ct,cr,rho=1.225,alpha='n',cl='n'):
        def chord(y,ct,cr):
            return 2*(ct-cr)*abs(y)+cr
        self.ylst=np.arange(-0.5,0.5,0.01)
        chord_lst=chord(self.ylst,ct,cr)
        if cl=='n':
            cl=self.interpolate(alpha)
        self.L_lst=0.5*rho*V**2*chord_lst*cl
    




            





if __name__ == "__main__":
    curves=lift_curve()
    #curves.interpolate(-4.2)
    alphalst=curves.alpha
    ind_lst=[]
    for i in range(len(alphalst)):
        ind_lst.append(curves.ind_drag(AR=8,e=0.85,h_b=0.050,alpha=alphalst[i]))
    
    fig=plt.figure()
    ax=fig.subplots(2,2)
    cl_lst=curves.cl_lst
    ax[0][0].set_title('Drag polar')
    ax[0][0].plot(alphalst,ind_lst)
    ax[0][1].set_title('lift curve')
    ax[0][1].plot(alphalst,cl_lst)
    L_D_lst=cl_lst/ind_lst
    ax[1][0].set_title('L/D')
    ax[1][0].set_ylim(0,70)
    ax[1][0].plot(alphalst,L_D_lst)
    curves.lift_dist(alpha=2,V=90,ct=4.5,cr=11.5)
    ax[1][1].set_title('spanwise lift distr')
    ax[1][1].plot(curves.ylst,curves.L_lst)
    ax[1][1].set_ylim(0,100000)
    plt.show()
    