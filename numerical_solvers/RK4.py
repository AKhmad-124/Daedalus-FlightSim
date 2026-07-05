def RK4Solver(f,y,h,t,*u):# where u is everything you need to compute f 
    k1 = f(t,y,*u)        # other than v like acc or torque or I for Soc
    k2 = f(t+h/2,y+k1*h/2,*u)
    k3 = f(t+h/2,y+k2*h/2,*u)
    k4 = f(t+h,y+k3*h,*u)
    kmean = (k1+2*k2+2*k3+k4)/6
    return  y + kmean*h