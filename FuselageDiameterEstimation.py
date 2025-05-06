import numpy as np

def deg2rad(deg):
    return deg * np.pi / 180


def CalcFuseDia(l,upsweep):
    d = (l+5.315/np.tan(deg2rad(upsweep)))/(2.5+1/np.tan(deg2rad(upsweep)))
    if d >= 5.315:
        return d
    else:
        return 5.315 #minimum needed to satisfy the rEqUirEmeNt of cargo hold area

def CalcFuseLen(l,upsweep,lfd, ltcd, lnd):   #length of cargo bay in "straight body region", upsweep of tail, lf/d, ltc/d, ln/d
    d = CalcFuseDia(l,upsweep)
    lf = lfd*d
    ltc = ltcd*d
    ln = lnd*d
    return d,lf,ltc,ln

DesignParams = [40, 11, 8.5, 4.5, 1.5] #follows same order as defined in function CalcFuseLen

FuseParam = CalcFuseLen(DesignParams[0],DesignParams[1],DesignParams[2], DesignParams[3], DesignParams[4])
print("Fuselage diameter:", FuseParam[0],"fuselage length:", FuseParam[1], "tail length:", FuseParam[2], "nosecone length:", FuseParam[3])


