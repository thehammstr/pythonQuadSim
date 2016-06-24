import numpy as np
from scipy import optimize
import matplotlib.pyplot as plt
from math import factorial


def Hmatrix(k,n,t):
  # Penalty matrix corresponding to the integral of the square of the kth derivative
  # over a given time interval t
  H = np.zeros((n+1,n+1))
  for i in range(k,n+1):
    for j in range(k,n+1):
      #print i,j
      H[i,j] =  ( 1./((i-k)+(j-k)+1)*
              (factorial(i)/factorial(i-k))*
              (factorial(j)/factorial(j-k))*(t**((i-k)+(j-k)+1)))

  return H


def Qmatrix(k,n,m,dim,t):
  # n is the order of the spline
  # k is the order of the derivative you're trying to minimize
  # m is the number of segments in the trajectory
  # t is the list of times to complete each segment of the trajectory ( len(t) = m )
  pass
  #Q = np.kron(np.eye(dim), Hmatrix(k,n,t[1]))
  Q = np.empty((0,0))
  for i in range(1,m+1):
    # replicate for each dimension
    Hm = np.kron(np.eye(dim), Hmatrix(k,n,t[i]-t[i-1]))
    Zm = np.zeros((Q.shape[0],Hm.shape[1]))
    ZmT = Zm.T
    Q = np.bmat('Q Zm; ZmT Hm').A
  return Q

def Arow(k,n,t):
  A = np.zeros((1,n+1));
  for i in range(k,n+1):
    A[0,i] = (factorial(i)/factorial(i-k))*t**(i-k)
  
  #print 'k, n, t: ',k,' ',n,' ', t
  return A

def queryPlan(t,n,dim,m,tSwitch,P,deriv=0):
  i = 0
  #print t,tSwitch[0]
  if m > 1:
    while i < m-1 and t > tSwitch[i+1]:
      #print i
      i += 1

  path = P[(i)*(n+1)*dim:(i+1)*(n+1)*dim]
  A = np.kron(np.eye(dim),Arow(deriv,n,t-tSwitch[i]))
  #print path
  return np.dot(A,path)

def fixedEndpointM(dim,m,n,waypoints,velocities):
  N = dim*m*(n+1)
  Nfixed = dim*(m+1)+2*dim*((n+1)/2 - 1)
  Nact = dim*((n+1)/2)*(m+1)
  Nfree = Nact - Nfixed
  M = np.zeros((N,Nact))
  #print M.shape
  
  # initial constraints
  M[0:(n+1)/2*dim,0:(n+1)/2*dim] = np.eye((n+1)/2*dim)
  # middle constraints
  Df = np.vstack((waypoints[0],velocities[0],np.zeros(((n+1)/2*dim - 2*dim,1))))
  for i in range(1,m):
    blocksize = (n+1)/2*dim
    Df = np.vstack((Df,waypoints[i]))
    for j in range(2):
      # handle pos
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize:j*(blocksize)+2*(i-1)*blocksize + blocksize + dim,
           blocksize+(i-1)*dim:blocksize+(i)*dim] = np.eye(dim)
  
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize+dim:j*(blocksize)+2*(i-1)*blocksize+blocksize + blocksize,
        Nfixed+(i-1)*(blocksize-dim):Nfixed+(i-1)*(blocksize-dim) + blocksize-dim] = np.eye(blocksize-dim)
  # final constraints
  M[N-(n+1)/2*dim:N,Nfixed-(n+1)/2*dim:Nfixed] = np.eye((n+1)/2*dim)
  Df = np.vstack((Df,waypoints[m],velocities[m],np.zeros(((n+1)/2*dim - 2*dim,1)) ))

  return [M, Df, Nfixed]

def floatEndpointM(dim,m,n,waypoints,velocities):
  N = dim*m*(n+1)
  # waypoints fixed and endpoint velocity
  Nfixed = dim*(m+1)+2*dim
  Nact = dim*((n+1)/2)*(m+1)
  Nfree = Nact - Nfixed
  M = np.zeros((N,Nact))
  #print M.shape
  blocksize = (n+1)/2*dim
  # initial constraints
  M[0:2*dim,0:2*dim] = np.eye(2*dim)
  M[2*dim:blocksize,Nfixed:Nfixed+blocksize-(2*dim)] = np.eye(blocksize-2*dim)
  # middle constraints
  Df = np.vstack((waypoints[0],velocities[0]))
  for i in range(1,m):
    Df = np.vstack((Df,waypoints[i]))
    for j in range(2):
      # handle pos
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize:j*(blocksize)+2*(i-1)*blocksize + blocksize + dim,
           (2*dim)+(i-1)*dim:(2*dim)+(i)*dim] = np.eye(dim)
  
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize+dim:j*(blocksize)+2*(i-1)*blocksize+blocksize + blocksize,
        (Nfixed+blocksize-2*dim)+(i-1)*(blocksize-dim):(Nfixed+blocksize-2*dim)+(i-1)*(blocksize-dim) 
        + blocksize-dim] = np.eye(blocksize-dim)
  # final constraints
  #print blocksize
  M[N-blocksize:N-(blocksize-2*dim),
      Nfixed-(2*dim):Nfixed] = np.eye(2*dim)
  M[N-(blocksize-2*dim):N,Nact-(blocksize-2*dim):Nact] = np.eye(blocksize-2*dim)
  Df = np.vstack((Df,waypoints[m],velocities[m] ))

  return [M, Df, Nfixed]

def specifyVelocitiesM(dim,m,n,waypoints,velocities):
  N = dim*m*(n+1)
  # waypoints fixed and endpoint velocity
  Nfixed = 2.0*dim*(m+1)
  Nact = dim*((n+1)/2)*(m+1)
  Nfree = Nact - Nfixed
  M = np.zeros((N,Nact))
  #print M.shape
  blocksize = (n+1)/2*dim
  # initial constraints
  M[0:2*dim,0:2*dim] = np.eye(2*dim)
  M[2*dim:blocksize,Nfixed:Nfixed+blocksize-(2*dim)] = np.eye(blocksize-2*dim)
  # middle constraints
  Df = np.vstack((waypoints[0],velocities[0]))
  for i in range(1,m):
    Df = np.vstack((Df,waypoints[i],velocities[i]))
    for j in range(2):
      # handle pos
      fixedx1 = j*(blocksize)+2*(i-1)*blocksize+blocksize 
      fixedx2 = j*(blocksize)+2*(i-1)*blocksize + blocksize + 2*dim
      fixedy1 = (2*dim)+2*(i-1)*dim
      fixedy2 = (2*dim)+2*(i)*dim
      
      M[fixedx1:fixedx2,fixedy1:fixedy2] = np.eye(2.0*dim)
      freex1 = j*(blocksize)+2*(i-1)*blocksize+blocksize+2.0*dim
      freex2 = j*(blocksize)+2*(i-1)*blocksize+blocksize + blocksize
      freey1 = (Nfixed+blocksize-2*dim)+(i-1)*(blocksize-2.0*dim)
      freey2 = (Nfixed+blocksize-2*dim)+(i)*(blocksize-2.0*dim)
      M[freex1:freex2,freey1:freey2] = np.eye(blocksize-(2.0*dim))
  # final constraints
  #print blocksize
  M[N-blocksize:N-(blocksize-2*dim),
      Nfixed-(2*dim):Nfixed] = np.eye(2*dim)
  M[N-(blocksize-2*dim):N,Nact-(blocksize-2*dim):Nact] = np.eye(blocksize-2*dim)
  Df = np.vstack((Df,waypoints[m],velocities[m] ))

  return [M, Df, Nfixed]




def generateTrajectory(waypoints, velocities,t,k = 3, n = 7, m = 1,dim=2):

  # Polynomial coefficients are packaged as:
  
  # P(t) = [px1 py1 pz1 px2 py2 pz2 ... pxm pym pzm]
  
  
  # Find the spline of order n that minimizes the integral of the square of the kth derivative
  H=Hmatrix(3,4,1)
  Q = Qmatrix(k,n,m,dim,t)
  
  '''Constraints: 
  
    Initial pos
    initial velocity
    all other initial derivatives zero
  '''
  A = np.empty((0,dim*m*(n+1)))
  d = np.empty((0,1))
  for seg in range(m):
    # first end
    #print A.shape, d.shape
    Ax0 = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), Arow(0,n,0.0) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
    dx0 = waypoints[seg]
    Axd0 = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), Arow(1,n,0.0) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
    dxd0 = velocities[seg]
    A = np.vstack((A,Ax0,Axd0))
    d = np.vstack((d,dx0,dxd0))
    for i in range(2,(n+1)/2):
      Ax = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), Arow(i,n,0.0) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
      A = np.vstack((A,Ax))
      d = np.vstack((d,np.zeros((dim,1))))
    # second end
    Ax0 = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), 
                      Arow(0,n,t[seg+1]-t[seg]) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
    dx0 = waypoints[seg+1]
    Axd0 = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), 
                      Arow(1,n,t[seg+1]-t[seg]) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
    dxd0 = velocities[seg+1]
    A = np.vstack((A,Ax0,Axd0))
    d = np.vstack((d,dx0,dxd0))
    for i in range(2,(n+1)/2):
      Ax = np.hstack( ( np.zeros((dim,seg*dim*(n+1))), np.kron( np.eye(dim), 
                        Arow(i,n,t[seg+1]-t[seg]) ) , np.zeros((dim,dim*(n+1)*(m-1-seg))) ))
      A = np.vstack((A,Ax))
      d = np.vstack((d,np.zeros((dim,1))))
  
  #print d.shape
  #print Q.shape
  
  #plt.imshow(A>0)
  #plt.show()

  Ainv = np.linalg.inv(A)
  #print Ainv
    
  '''
  M = np.zeros((N,Nact))
  #print M.shape
  
  # initial constraints
  M[0:(n+1)/2*dim,0:(n+1)/2*dim] = np.eye((n+1)/2*dim)
  # middle constraints
  Df = np.vstack((waypoints[0],velocities[0],np.zeros(((n+1)/2*dim - 2*dim,1))))
  for i in range(1,m):
    blocksize = (n+1)/2*dim
    Df = np.vstack((Df,waypoints[i]))
    for j in range(2):
      # handle pos
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize:j*(blocksize)+2*(i-1)*blocksize + blocksize + dim,
           blocksize+(i-1)*dim:blocksize+(i)*dim] = np.eye(dim)
  
      M[j*(blocksize)+2*(i-1)*blocksize+blocksize+dim:j*(blocksize)+2*(i-1)*blocksize+blocksize + blocksize,
        Nfixed+(i-1)*(blocksize-dim):Nfixed+(i-1)*(blocksize-dim) + blocksize-dim] = np.eye(blocksize-dim)
  # final constraints
  M[N-(n+1)/2*dim:N,Nfixed-(n+1)/2*dim:Nfixed] = np.eye((n+1)/2*dim)
  Df = np.vstack((Df,waypoints[m],velocities[m],np.zeros(((n+1)/2*dim - 2*dim,1)) ))
  '''
  #[M,Df,Nfixed] = fixedEndpointM(dim,m,n,waypoints,velocities)
  [M,Df,Nfixed] = specifyVelocitiesM(dim,m,n,waypoints,velocities)
  #[M,Df,Nfixed] = floatEndpointM(dim,m,n,waypoints,velocities)

  #print Df
  
  R = np.dot(M.T,np.dot(np.dot(np.dot(Ainv.T,Q),Ainv),M))
  #print Q.shape

  #print R
  
  Rpp = R[Nfixed:,Nfixed:].copy()
  Rfp = R[0:Nfixed,Nfixed:].copy()
  #print Rfp
  
  dStar = np.dot(np.dot(-np.linalg.inv(Rpp),Rfp.T),Df)
  
  D = np.vstack((Df,dStar))
  pOpt = np.dot(Ainv,np.dot(M,D))
  
  return [pOpt, Q]

def J(x,waypoints,velocities,k,n,m,aggression):

  times = x
  times = np.hstack((times,0))
  times[0] = 0.
  for i in range(1,times.shape[0]):
    times[i] = times[i-1]+x[i-1]

  [pOpt,Q] = generateTrajectory(waypoints, velocities,times,k, n, m)

  #print times[-1], x, np.dot(np.dot(pOpt.T,Q),pOpt)[0,0],  aggression*times[-1], np.dot(np.dot(pOpt.T,Q),pOpt)[0,0] + aggression*times[-1]

  return np.dot(np.dot(pOpt.T,Q),pOpt)[0,0] + aggression*times[-1]
  

def generateOptimalTrajectory(waypoints, velocities,times,k, n, m, aggression):
  '''
  print 'k ', k
  print 'n ', n
  print 'm ', m
  print 'waypoints ', waypoints
  print 'velocities ', velocities
  print 'times ', times
  '''
  x0 = np.diff(times)


  bnds = ((1., None),)
  for i in range(1,m):
    bnds += ((0.1,None),)
  #print bnds

  res = optimize.minimize(J,x0,args=(waypoints,velocities,k,n,m,aggression),bounds=bnds)
  #res = optimize.minimize(J,x0,args=(waypoints,velocities,k,n,m,aggression))

  #print res.x

  times = res.x
  times = np.hstack((times,0))
  times[0] = 0.
  for i in range(1,times.shape[0]):
    times[i] = times[i-1]+res.x[i-1]

  return times



def main():

  # START MAIN CODE
  np.set_printoptions(precision=5,linewidth=120)
  '''
  print "pOpt:"
  print pOpt
  '''
  
  k = 3 # min jerk = 3
  n = 5 # Order of polynomial for representing position. needs to be odd
  m = 4 # how many path segments
  dim = 2 # 2-Dhorizontal plane
  
  x_0 = np.array([[0.,0.]]).T
  x_1 = np.array([[15.,0.]]).T
  x_2 = np.array([[19.,1.0]]).T
  x_3 = np.array([[20.,5.]]).T
  x_4 = np.array([[20.,20.]]).T
  
  
  v_i = np.array([[5.,0.]]).T
  #v_1 = np.array([[1.0,0.0]]).T # Don't constrain intermediate vel
  v_f = np.array([[0.0,5.0]]).T
  
  waypoints =  [x_0,x_1, x_2, x_3,x_4]
  times = [0.,3.,4,5.,8.]
  #waypoints =  [x_0, x_1, x_3]
  #times = [0.,  3.06 , 5.3]
  
  velocities = [v_i, np.array([[0.0,0.0]]).T , np.array([[0.0,0.0]]).T,np.array([[0.0,0.0]]).T, v_f]
  #velocities = [v_i, np.array([[0.0,0.0]]).T , v_f]
  '''
  print waypoints
  print velocities
  print times
  '''
  aggression = 10. # Higher = faster & larger derivatives

  t1 = time.time()
  optimalTimes = generateOptimalTrajectory(waypoints, velocities,times,k, n, m, aggression)
  #optimalTimes = times
  print "elapsed time: ", time.time()-t1
  [pOpt,Q] = generateTrajectory(waypoints, velocities,optimalTimes,k, n, m, dim)
  #[pOpt,Q] = generateTrajectory(waypoints, velocities,times,k, n, m, dim)
  
  print "elapsed time: ", time.time()-t1

  print times
  print optimalTimes
  
  x = np.empty((dim,0))
  
  Times = np.linspace(0,optimalTimes[-1],num=200)
  for T in Times:
    x = np.hstack((x,queryPlan(T,n,dim,m,optimalTimes,pOpt)))
  
  plt.plot(x[0,:],x[1,:])
  
  for wp in waypoints:
    plt.scatter(wp[0,0],wp[1,0])
  
  TimesVel = np.linspace(0,optimalTimes[-1],num=30)
  for T in TimesVel:
    pos = queryPlan(T,n,dim,m,optimalTimes,pOpt,0)
    vel = queryPlan(T,n,dim,m,optimalTimes,pOpt,1)
    plt.quiver(pos[0,0],pos[1,0],vel[0,0],vel[1,0])
  plt.grid(True)
  plt.axis('equal')
  plt.show()


if __name__ == "__main__":
    main()

