import numpy as np
import matplotlib.pyplot as plt
def f(x):
    return x**2

class aero_centre_intergration():
    def  __init__(self):
        self.import_data()
    
    def calc(self):
        self.constr_Au()
        self.calc_w()
        self.calc_I()

    def import_data(self):
        self.data=np.loadtxt("aero\\wing distr.txt")
        print(self.data)
        self.pos=self.data[:,0]
        #print(self.cl)
        print(self.pos)
        self.n=int(self.pos.shape[0])*2
        #self.pos=self.data[int(self.n/2):self.n,0]
        #print(self.pos)
        self.cl=self.data[:,3]
        self.b=self.pos[-1]
        self.a=self.pos[0]
        print(self.pos)
        
    
    def constr_Au(self):
        self.A=np.array([np.array([None]*int(self.n/2),dtype=float)]*int(self.n/2),dtype=float)
        self.u=np.array([None]*int(self.n/2),dtype=float)
        #print(A)
        for i in range(int(self.n/2)):
            #print(A)
            self.A[i,:]=self.pos**i
            self.u[i]=(self.b**(i+1)-self.a**(i+1))/(i+1)
        print(self.A)
        print(self.pos)
        print(self.u)
        print(self.A.shape)
        print(self.u.shape)
    
    def calc_w(self):
        self.w=np.linalg.solve(self.A,self.u)
        print(f'hi {self.w}')
        #self.w=self.u@np.linalg.inv(self.A)
        #print(self.w)

    def calc_I(self):
        I_1=0
        I_2=0
        for i in range(int(self.n/2)):
            I_1=I_1+self.w[i]*self.cl[i]*self.pos[i]
            I_2=I_2+self.w[i]*self.cl[i]
        
        self.I=I_1/I_2
        print(I_2)
        print(self.I)
        return self.I
    
    def plot(self):
        fig=plt.figure()
        ax=fig.add_subplot()
        ax.plot(self.pos,self.cl)
        ax.vlines(self.I,min(self.cl),max(self.cl), linestyles='dashed', colors='red')
        plt.show()


        



if __name__ == '__main__':
    obj=aero_centre_intergration()
    obj.calc()
    obj.plot()


