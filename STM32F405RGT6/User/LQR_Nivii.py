Qlqr=np.identity(4)
Rlqr=np.identity(2)
Pf=np.asarray([[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,0.]])
t=160
def LQRP(A,B,R,Q,Pf,t):
    P_=[Pf]
    j=0
    t_=[0]
    while j*dt < t:
        t_.append((j+1)*dt)
        W=P_[-1]+np.dot(P_[-1],A*dt)+np.dot(A.T*dt,P_[-1])-np.dot(P_[-1],np.dot(B,np.dot(R,np.dot(B.T*dt,P_[-1]))))+(Q*dt)
        P_.append(W)
        j += 1
    return np.asarray(t_),np.asarray(P_)
Ty_,PARE_=LQRP(A,B,Rlqr,Qlqr,Pf,t)
def LQRu(x,P):
    W=np.dot(-1*Rlqr,np.dot(B.T,np.dot(P,x)))
    return W
def simLQR(f,t,x):  #simulation function
    j,t_,x_,u_ = 0,[0],[x],[]
    count=np.size(Ty_)-1
    while j*dt < t:
        t_.append((j+1)*dt)
        u_.append(LQRu(x_[-1],PARE_[count]))
        x_.append(x_[-1] + dt*f(j*dt,x_[-1],u_[-1]))
        j += 1
        count -=1
    return np.asarray(t_),np.asarray(x_),np.asarray(u_)
