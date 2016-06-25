import optimalPolyTraj as tj
import matplotlib.pyplot as plt
import numpy as np

k = 3 # the derivative you're minimizing. k=3 is jerk
n = 5 # the order of the polynomials. For now, the code only allows odd values.
m = 6 # the number of path segments
dim = 2 # the dimension of your state
# times at which you reach waypoints (initial guess)
# good idea to start with times based on distance between points.
times = [0,1,4.5,5.,5.5,6.5,8] 

scale = 2.0
x_0 = scale*np.array([[0.,0.]]).T
x_1 = scale*np.array([[15.,0.]]).T
x_11 = scale*np.array([[(50+15)/2.,0.]]).T
x_2 = scale*np.array([[50.,0.0]]).T
x_3 = scale*np.array([[65.,0.]]).T
x_4 = scale*np.array([[70.,5.]]).T
x_5 = scale*np.array([[70.,20.]]).T
#x_4 = np.array([[20.,20.]]).T

waypoints = [x_0,x_1,x_11,x_2,x_3,x_4,x_5]
v_0 = np.array([[3.,0.]]).T
v_1 = np.array([[10.,0.]]).T
v_11 = np.array([[10.,0.]]).T
v_2 = np.array([[10.,0.]]).T
v_3 = np.array([[3.,0.]]).T
v_4 = np.array([[0.,3.]]).T
v_5 = np.array([[0.,10.]]).T
#v_1 = np.array([[1.0,0.0]]).T # Don't constrain intermediate vel
vzero = np.zeros((dim,1))
#v_f = np.array([[0.0,3.0]]).T

velocities = [v_0,v_1,v_11,v_2,v_3,v_4,v_5]

[M,Df,Nfixed] = tj.specifyVelocitiesM(dim,m,n,waypoints,velocities)
#print M

#plt.imshow(M>0)
#plt.show()

aggression = 1. # Higher = faster & larger derivatives

optimalTimes = tj.generateOptimalTrajectory(waypoints, velocities,times,k, n, m, aggression)
#optimalTimes = times
[pOpt,Q] = tj.generateTrajectory(waypoints, velocities,optimalTimes,k, n, m, dim)


print times
print optimalTimes

x = np.empty((dim,0))
v = np.empty((dim,0))
acc = np.empty((dim,0))

Times = np.linspace(0,optimalTimes[-1],num=100)
for T in Times:
  x = np.hstack((x,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt)))
  v = np.hstack((v,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,1)))
  acc = np.hstack((acc,tj.queryPlan(T,n,dim,m,optimalTimes,pOpt,2)))

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

print v.T
plt.plot(v.T)
plt.plot(acc.T,'-')
plt.show()

