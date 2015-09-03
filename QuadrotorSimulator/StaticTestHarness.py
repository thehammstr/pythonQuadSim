#! /usr/bin/python
import numpy as np
import scipy as sc
import math
import Motor
import Propeller

# create motor
M = Motor.Motor()
stateHist = np.zeros((4000,2))
idx = 0
dT = 0.001
# command motor
M.commandMotor(.8)
# Add wind
VehVel = np.zeros((3,1))
VehVel[0,0] = 3
VehVel[1,0] = 2
# run for a while
for jj in [0,1]:
    for ii in range(0,2000):
        y = M.updateState(dT,windVelocity=-VehVel)
        stateHist[idx,:] = y
        idx = idx+1
    M.commandMotor(.2)

print stateHist
print y


import matplotlib.pyplot as plt

fig1 = plt.figure()
plt.plot(stateHist[:,1])
plt.plot(stateHist[:,0])
plt.show()
