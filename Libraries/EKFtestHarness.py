#! /usr/bin/python




import numpy as np
import AeroQuaternion as AQ
import KinematicEKF as kEKF

filter = kEKF.kinematicQuadEKF()

accelerometer = np.array([[.01,.01,-9.82]]).T
gravMeas =  np.array([[.0,.0,-9.81]]).T
gyro = np.array([[.01,.01,.01]]).T
velocity = np.zeros((3,1))
pos = velocity
dT = 0.01
for ii in range(0,100):

   pos = pos + velocity*dT
   velocity = velocity + (accelerometer-gravMeas)*dT

   gps = ['gps',pos,np.eye(3)]
   other = [gps]

   stateAndCov = filter.runFilter(accelerometer,gyro,other,dT)

   print 'state: '
   print stateAndCov[0].T

