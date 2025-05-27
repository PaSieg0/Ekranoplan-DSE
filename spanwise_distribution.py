import numpy as np


class lift_distribution:
    def __init__(self, S, AR, cl_alpha, c1=0.0, c2=0.0, c3=0.0, c4=0.0, ):
        self.S = S
        self.AR = AR
        self.cl_alpha  = cl_alpha
        self.c1 = c1 #can change
        self.c2 = c2 #can change
        self.c3 = c3 #can change
        self.c4 = c4 #can change

    def geo_chord(self):
        """
        Calculate the geometric chord of the wing.
        :return: Geometric chord (m)
        """
        return np.sqrt(self.S / self.AR)
    
    def F(self):
        return 2* np.pi * self.AR / self.cl_alpha
    
    



    
