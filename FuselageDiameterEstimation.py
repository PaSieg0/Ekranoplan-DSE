import numpy as np

def deg2rad(deg):
    return deg * np.pi / 180


def CalcFuseDia(l,upsweep):
    d = (l+5.315/np.tan(deg2rad(upsweep)))/(2.5+1/np.tan(deg2rad(upsweep)))
    if d >= 5.315:
        return d
    else:
        return 5.315 #minimum needed to satisfy the rEqUirEmeNt of cargo hold area

def CalcFuseLen(l,upsweep,lfd):
    d = CalcFuseDia(l,upsweep)
    lf = lfd*d
    return d,lf

DesignParams = [20, 11, 8.5] #length of cargo bay in "straight body region", upsweep of tail, lf/d

FuseParam = CalcFuseLen(DesignParams[0],DesignParams[1],DesignParams[2])
print("Fuselage diameter:", FuseParam[0],"fuselage length:", FuseParam[1])


