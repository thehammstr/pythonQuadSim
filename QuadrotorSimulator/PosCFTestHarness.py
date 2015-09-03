#! /usr/bin/python
import numpy as np
import scipy as sc
import math
import Multirotor
import AeroQuaternion as Quat
import QuadrotorController
import KinematicEKF
import time as clock
import AeroQuaternion as AQ
# ZMQ shit
# Import ZMQ, our fancy-pants networking thingy
import zmq

# Import our custom "state_pb2" protobuf-python code
from protobuf.py import state_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

# Create our Publishing socket here
socket = context.socket(zmq.PUB)
socketEKF = context.socket(zmq.PUB)
# Bind our publishing socket to localhost on some port
socket.bind("tcp://*:5000")
socketEKF.bind("tcp://*:5001")

def publishState(time,state):
        pose = state_pb2.PosePb()
        pose.name = 'SimulatedPose'
        pose.valid = True
        pose.timestamp.seconds = time; # need real timestamp eventually
        Q1 = Quat.Quaternion(np.array([180,0,0]))
        pose.translation.x = state[0,0]
        pose.translation.y = state[0,1]
        pose.translation.z = state[0,2] 
        Q = Quat.Quaternion(state[0,6:10])
        xform = Q #Q1*Q
        #pose.rotation.x = state[0,6]
        #pose.rotation.y = state[0,7]
        #pose.rotation.z = state[0,8]
        #pose.rotation.w = state[0,9]
        pose.rotation.x = xform.q[0]
        pose.rotation.y = xform.q[1]
        pose.rotation.z = xform.q[2]
        pose.rotation.w = xform.q[3]
        socket.send( pose.SerializeToString() )

def publishEstimatedState(time,state):
        pose = state_pb2.PosePb()
        pose.name = 'EstimatedPose'
        pose.valid = True
        pose.timestamp.seconds = time; # need real timestamp eventually
        Q1 = Quat.Quaternion(np.array([180,0,0]))
        pose.translation.x = state[0,0]
        pose.translation.y = state[0,1]
        pose.translation.z = state[0,2] 
        Q = Quat.Quaternion(state[0,6:10])
        xform = Q 
        #pose.rotation.x = state[0,6]
        #pose.rotation.y = state[0,7]
        #pose.rotation.z = state[0,8]
        #pose.rotation.w = state[0,9]
        pose.rotation.x = xform.q[0]
        pose.rotation.y = xform.q[1]
        pose.rotation.z = xform.q[2]
        pose.rotation.w = xform.q[3]

        socketEKF.send( pose.SerializeToString() )



# create quadrotor
Quad = Multirotor.Multirotor() # default is quadrotor
idx = 0
dt = 0.005
T = 1.3
numsteps = 3
maxInd = int(math.ceil(T/dt))
stateHist = np.zeros((numsteps*maxInd,13))
# command motor
commands = [.5,.5,.5,.5]
#commands = [0,0,0,0]
# add wind
windvel = np.zeros((3,1))
# run for a while
controller = QuadrotorController.Controller()
EKF = KinematicEKF.kinematicQuadEKF()
PosFilt = KinematicEKF.PositionComplementaryFilter();
AttEstimator = KinematicEKF.AttitudeComplementaryFilter()
reference = [0.,0.,0.,3.]
time = 0.
period = dt
'''
for jj in range(int(numsteps)):
    for ii in range(0,maxInd):
        state = Quad.updateState(dt,commands,windVelocity=windvel)
        #stateHist[idx,:] = state
        idx = idx+1
        publishState(idx*dt,state)
    if(jj <= 1):
        pass
        commands = [.3,.3, .3, .4]
        #commands = [0, 0, 0, 0]
    else:
        #commands = [.5,.5,.5,.5]
        commands = [0,0,0,0]
print stateHist
'''
Quad.stateVector[0,2] = 0. # initial height
np.random.seed([])
gravTracker = 0.
accMeas = np.zeros((3,1))
lastUpdateTime = 0.
#mountError = AQ.Quaternion(np.array([3.,1.,0]))
mountError = AQ.Quaternion(np.array([0.,0.,0]))
while(True):
    startTime = clock.time()
    windvel = windvel + np.random.randn(3,1)
    disturbance = 10
    state,acc = Quad.updateState(dt,commands,windVelocity = windvel,disturbance = disturbance)
    #print 'norm acc: ', np.linalg.norm(acc)
    accMeas = acc + .001*np.array([np.random.randn(3)]).T + np.array([[.0],[0],[0]]) 
    gravTracker = .99*gravTracker + .01*acc[2,0]
    #print 'gravity ', gravTracker
    # run EKF
    gyroMeas = state.T[10:] + .001*np.array([np.random.randn(3)]).T + np.array([[0.0],[0],[0]]) #+ np.array([[0.],[0.01],[.0]])# + .001*np.ones((3,1))
    posMeas = state.T[0:3] + .01*np.array([np.random.randn(3)]).T
    attTrue = AQ.Quaternion(state[0,6:10])
    magMeas = np.dot(attTrue.asRotMat,np.array([[1.,0.,0.]]).T)
    if (time-lastUpdateTime > .2):
       otherMeas = []
       otherMeas.append(['gps',posMeas,100.*np.eye(3)])
       otherMeas.append(['mag',magMeas,1.*np.eye(3)])
       otherMeas.append(['barometer',posMeas[2,0]+.01*np.random.rand(),2*np.ones((1,1))])
       lastUpdateTime = time
    else:
       otherMeas = []
    # Modified for MEKF
    attitudeAndGyroBias = AttEstimator.runFilter(accMeas,gyroMeas,[],dt)
    startKF = clock.time()
    stateAndCov = EKF.runFilter(accMeas,gyroMeas-attitudeAndGyroBias[1],mountError*attTrue,otherMeas,dt)
    stateAttitude = EKF.qReference*AttEstimator.attitudeEstimate
    estimatedPosition,estimatedVelocity,accBias = PosFilt.runFilter(np.dot(stateAttitude.asRotMat.T,accMeas),otherMeas,dt)
    print 'kfTime: ',clock.time() - startKF
    print 'pos: ', estimatedPosition.T, state[0,0:3]
    print 'vel: ', estimatedVelocity.T, state[0,3:6]
    print 'eulers',stateAttitude.asEuler, attTrue.asEuler
    print 'accCor: ', accBias.T 
    #stateAttitude = AQ.Quaternion(np.array([0,0,0]))
    #stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([stateAttitude.q]).T])
    #stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([(stateAttitude*mountError*attTrue).q]).T])
    stateToPublish = np.vstack([estimatedPosition,estimatedVelocity, np.array([stateAttitude.q]).T])
    controlState = np.vstack([stateToPublish,gyroMeas-attitudeAndGyroBias[1]])
    #print 'actual attitude ',
    #print attTrue.asEuler
    #print 'attitude est: ', attitudeAndGyroBias[0].asEuler, 'gyro bias: ', attitudeAndGyroBias[1].T
    #print 'covariance'
    #print stateAndCov[1]
    if (time < 7):
        reference = [0,0,3,0]
    elif (time < 11):
        reference = [0,0,3,np.pi/2-.1]
    else:
        reference = [1.,0.,3.,0.]
    #commands = controller.updateControl(dt,state,reference,'xyah')
    commands = controller.updateControl(dt,controlState.T,reference,'xyah')
    #commands = controller.updateControl(dt,controlState.T,[.0,.0,.2,.4],'rpYRt')
    publishState(time,state)
    publishEstimatedState(time,stateToPublish.T)
    time = time + dt
    print 'time: ',time
    if (time>17. and time < 21.5):
       windvel = np.array([[3,0,0]]).T
       #windvel = np.array([[0,0,0]]).T
    elif (time>8.5 and time < 11.5):
       #windvel = np.array([[-3,0,0]]).T
       windvel = np.array([[0,0,0]]).T
    else:
       windvel = np.zeros((3,1))
    '''
    if (np.floor(time) % 2 == 0): # and np.floor(time) < 10):
       reference = [0,-.1,0,3]
    #elif (np.floor(time) > 10 and np.floor(time) < 12):
    #   reference = [0,.2,0,3]
    elif(time > 10.):
       reference = [0,0,0,3]
    else: 
       reference = [0,.1,0,3]

    #elif (np.floor(time) % 6 == 3):
    #   reference = [0,0,np.pi/4,3.] 
    # kill time to make realtime
    '''
    while(clock.time() - startTime < dt):
       pass
import matplotlib.pyplot as plt

fig1 = plt.figure()
for ii in range(13):
    plt.plot(stateHist[:,ii])
#plt.plot(stateHist[:,5])
plt.show()





