import polynomialTrajectories as tj
import matplotlib.pyplot as plt
import numpy as np

k = 3 # the derivative you're minimizing. k=3 is jerk
n = 7 # the order of the polynomials. For now, the code only allows odd values.
m = 1 # the number of path segments
dim = 2 # the dimension of your state
# times at which you reach waypoints (initial guess)
# good idea to start with times based on distance between points.
times = [0,10.5]#,5.,5.5,6.5,8]

scale = 1.0
x_0 = scale*np.array([[0.,0.]]).T
x_1 = scale*np.array([[40.,0.]]).T
#x_11 = scale*np.array([[(50+15)/2.,0.]]).T
#x_2 = scale*np.array([[50.,0.0]]).T
#x_3 = scale*np.array([[65.,0.]]).T
#x_4 = scale*np.array([[70.,5.]]).T
#x_5 = scale*np.array([[70.,20.]]).T
#x_4 = np.array([[20.,20.]]).T

waypoints = [x_0,x_1]#,x_11,x_2,x_3,x_4,x_5]
v_0 = np.array([[1.0,0.]]).T
v_1 = np.array([[0.,0.]]).T
#v_11 = np.array([[10.,0.]]).T
#v_2 = np.array([[10.,0.]]).T
#v_3 = np.array([[3.,0.]]).T
#v_4 = np.array([[0.,3.]]).T
#v_5 = np.array([[0.,10.]]).T
#v_1 = np.array([[1.0,0.0]]).T # Don't constrain intermediate vel
vzero = np.zeros((dim,1))
#v_f = np.array([[0.0,3.0]]).T

velocities = [v_0,v_1]#,v_11,v_2,v_3,v_4,v_5]

[M,Df,Nfixed] = tj.zeroInitialAccelM(dim,m,n,waypoints,velocities)

#plt.pcolor(M)
#plt.show()

aggression = .05 # Higher = faster & larger derivatives

optimalTimes = tj.generateOptimalTrajectory(waypoints, velocities,times,k, n, m, aggression)
#optimalTimes = times
[pOpt,Q] = tj.generateTrajectory(waypoints, velocities,optimalTimes,k, n, m, dim)


print times
print optimalTimes

x = np.empty((dim,0))
v = np.empty((dim,0))
acc = np.empty((dim,0))
jrk = np.empty((dim,0))

Times = np.linspace(0,optimalTimes[-1],num=100)
for T in Times:
  x = np.hstack((x,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt)))
  v = np.hstack((v,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,1)))
  acc = np.hstack((acc,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,2)))
  jrk = np.hstack((jrk,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,3)))

plt.plot(x[0,:],x[1,:])

for wp in waypoints:
  plt.scatter(wp[0,0],wp[1,0])

TimesVel = np.linspace(0,optimalTimes[-1],num=20)
for T in TimesVel:
  pos = tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,0)
  vel = tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,1)
  plt.quiver(pos[0,0],pos[1,0],vel[0,0],vel[1,0])
plt.grid(True)
plt.axis('equal')
plt.show()

plt.clf()

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax1.plot(x.T[:,0],v.T[:,0],'r',label="vel")
ax1.plot(x.T[:,0],acc.T[:,0],'g',label="acc")
ax1.plot(x.T[:,0],jrk.T[:,0],'b',label="jrk")
plt.title("Derivatives vs distance and time")
plt.xlabel("Distance")
handles, labels = ax1.get_legend_handles_labels()
ax1.legend(handles, labels)
ax2 = fig.add_subplot(212)
ax2.plot(Times,v.T[:,0],'r',label="vel")
ax2.plot(Times,acc.T[:,0],'g',label="acc")
ax2.plot(Times,jrk.T[:,0],'b',label="jrk")
plt.xlabel("Time")
handles, labels = ax2.get_legend_handles_labels()
ax2.legend(handles, labels)


plt.show()

