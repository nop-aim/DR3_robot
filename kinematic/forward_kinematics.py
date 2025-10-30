import numpy as np
import kinematic.DH_modify as DH
def forward_kinematic(q1,q2,q3,degree):
    # === Thông số robot ===
    L1 = 180
    L2 = 244
    L3 = 269
    d1 = 120
    d2 = 93
    #Tính toán ma trận biến đổi đồng nhất từ base đến end-effector
    #TB0=DH.DH_modify(0,0,0,0)
    T01=DH.DH_modify(0,0,q1,L1,degree)
    T12=DH.DH_modify(90,0,q2,-d1,degree)
    T23=DH.DH_modify(0,L2,q3,d2,degree)
    T3EE=DH.DH_modify(0,L3,0,0,degree)
    
    T0EE=T01@T12@T23@T3EE
    
    return T0EE