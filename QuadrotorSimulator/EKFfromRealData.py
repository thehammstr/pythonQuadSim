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
import matplotlib.pyplot as plt
import collections
import KBeaconBridge

# ZMQ shit
# Import ZMQ, our fancy-pants networking thingy
import zmq

# Import our custom "state_pb2" protobuf-python code
from protobuf.py import state_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

# Create subscriber sockets
IMUReceiver = context.socket(zmq.SUB)
IMUReceiver.connect(KBeaconBridge.lookup_blocking("replay_drone.imu"))
IMUReceiver.setsockopt(zmq.SUBSCRIBE, "")

VICONReceiver = context.socket(zmq.SUB)
VICONReceiver.connect(KBeaconBridge.lookup_blocking("replay_drone.vicon_pose_proxy"))
VICONReceiver.setsockopt(zmq.SUBSCRIBE, "")

# Create our Publishing socket here
socketEKF = context.socket(zmq.PUB)
# Bind our publishing socket to localhost on some port
socketEKF.bind("tcp://*:5001")
KBeaconBridge.publish("replay_drone.ekf_pose", "drone.pose", "5001");


#accCalibMat = np.array(
#[[  9.96645001e-01,  -2.81384227e-03,   8.01922199e-04],
# [ -2.81384227e-03,  1.01852188e+00,  -1.16079997e-02],
# [  8.01922199e-04,  -1.16079997e-02,   9.97722770e-01]])
 
#accoffset = np.array(
#[[ 0.03551065],
# [-0.046777  ],
# [-0.03862092]])

# plots
#plt.ion()             # Interactive plotting mode
# Deques remove old data when new data is appended (essentially a circular buffer)
biasPlotX = collections.deque([0]*1000, 1000)
biasPlotY = collections.deque([0]*1000, 1000)
biasPlotZ = collections.deque([0]*1000, 1000)
biasPlotW = collections.deque([0]*1000, 1000)
time = np.linspace(0,.005*1000,1000)
# Plot commands
line1, = plt.plot(time, biasPlotX, label = "commanded current (A)")
line2, = plt.plot(time, biasPlotY, label = "output (PWM)")
line3, = plt.plot(time, biasPlotZ, label = "RMS current (A)")
line4, = plt.plot(time, biasPlotW, label = "RMS current (A)")
#plt.xlabel("Time (s)")
#plt.ylabel("bias");


def processIMUData(IMUdata,useMag = False):
        time = IMUdata.time_ms
        otherMeas = []
        if (IMUdata.HasField("gyro")):
           gx = IMUdata.gyro.x
           gy = IMUdata.gyro.y
           gz = IMUdata.gyro.z
           gyroOut = np.array([[gx],[gy],[gz]])
        else:
           return [False,0,0,0]
        if (IMUdata.HasField("accel")):
           ax = IMUdata.accel.x
           ay = IMUdata.accel.y
           az = IMUdata.accel.z
           #accOut = 9.81*np.array([[ax],[ay],[az]])#9.81*np.dot(accCalibMat,np.array([[ax],[ay],[az]]) - accoffset)
           #accOut = 9.81*np.dot(accCalibMat,np.array([[ax],[ay],[az]]) - accoffset)
           accOut = 9.81*np.array([[ax],[ay],[az]])
        else:
           return [False,0,0,0]
        if (IMUdata.HasField("pose")):
           qx = IMUdata.pose.rotation.x
           qy = IMUdata.pose.rotation.y
           qz = IMUdata.pose.rotation.z
           qw = IMUdata.pose.rotation.w
           attitude = AQ.Quaternion(np.array([qx, qy, qz, qw]))
        if (IMUdata.HasField("mag") and useMag):
           mx = IMUdata.mag.x
           my = IMUdata.mag.y
           mz = IMUdata.mag.z
           magOut = np.array([[mx],[my],[mz]])
           otherMeas.append(['mag',magOut,1*np.eye(3)])
        return [True,time,gyroOut, accOut, attitude, otherMeas]

def processVICONData(VICONdata):
        if VICONdata.valid:
           x = VICONdata.translation.x
           y = VICONdata.translation.y
           z = VICONdata.translation.z
           measPos = np.array([[x],[y],[z]]) + .1*np.array([np.random.randn(3)]).T
           qx = VICONdata.rotation.x
           qy = VICONdata.rotation.y
           qz = VICONdata.rotation.z
           qw = VICONdata.rotation.w
           attitude = AQ.Quaternion(np.array([qx,qy,qz,qw]))
           spoofMagMeas = np.dot(attitude.asRotMat,np.array([[1,0,0]]).T)
           return [True, [['gps',measPos,1*np.eye(3)],['mag',spoofMagMeas,.01*np.eye(3)]]]
        else:
           return [False,[]]
     

def publishEstimatedState(time,state):
    x,y,z,u,v,w,qx,qy,qz,qw,p,q,r = state[0]
    
    pose = state_pb2.PosePb()
    pose.name = 'SimulatedPose'
    pose.valid = True
    pose.timestamp.seconds = time; # need real timestamp eventually
    Q1 = Quat.Quaternion(np.array([180,0,0]))
    pose.translation.x = x
    pose.translation.y = y
    pose.translation.z = z
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
    
    #vel_inertial = Quat.rotateVector(Q.inv(),state[0,3:6]).T
    pose.velocity.x = state[0,3]
    pose.velocity.y = state[0,4]
    pose.velocity.z = state[0,5]
    
    pose.rotational_velocity.x = state[0,10]
    pose.rotational_velocity.y = state[0,11]
    pose.rotational_velocity.z = state[0,12]
    
    socketEKF.send( pose.SerializeToString() )


############################################################################
# here's the real script
############################################################################

EKF = KinematicEKF.kinematicQuadEKF()
AttitudeFilt = KinematicEKF.AttitudeComplementaryFilter()
lastTime = 0.

# Set up ZMQ subscriber/poller stuff
poller = zmq.Poller()
poller.register(IMUReceiver, zmq.POLLIN)
#poller.register(VICONReceiver, zmq.POLLIN)
IMUdata = state_pb2.ImuPb()
VICONdata = state_pb2.PosePb()
firstPass = True
firstPlot = True
ii = 0
jj = 0
while(True):
    print 'loop!'
    # read in data from zmq
    socks = dict(poller.poll())

    if (IMUReceiver in socks and socks[IMUReceiver] == zmq.POLLIN):
        IMUdata.ParseFromString(IMUReceiver.recv())
        processedIMUdata = processIMUData(IMUdata,useMag=False)
  #  if (VICONReceiver in socks and socks[VICONReceiver] == zmq.POLLIN):
    receivedVicon = False
    flyingFlag = False
    while(True):
      try:
        data = VICONReceiver.recv(flags=zmq.NOBLOCK)
        VICONdata.ParseFromString(data)
        receivedVicon = True
        #print 'VICON!'
      except:
        'EXCEPTION in VICON!'
        break
    # end loop
    if (receivedVicon):
      if (ii >= 20):
       ii = 0 #slow down vicon
       processedVICONdata = processVICONData(VICONdata)
       validVicon,viconMeas = processedVICONdata
       if (validVicon and viconMeas[0][1][2,0] < -.5):
          flyingFlag = True
       elif (validVicon):
          flyingFlag = False
       otherMeas = viconMeas
      else:
       ii = ii + 1
       otherMeas = []
    else:
       otherMeas = []
 
    dataValid,currentTime,gyroMeas,accMeas,attitudeIMU,otherMeasurements = processedIMUdata

    #print 'gyro: ',gyroMeas.T
    #print 'acc: ', accMeas.T
    #print 'other ',otherMeas
    #print 'time: ',currentTime
    if(dataValid):
       if(firstPass):
          dT = 0.005
          lastTime = currentTime
          firstPass = False
       else:
          dT = (currentTime - lastTime)/1000.
          lastTime = currentTime

       # run attitude filter
       attitude,gyroBiasEst = AttitudeFilt.runFilter(accMeas-EKF.state[9:12],gyroMeas,otherMeas,dT)
       # run high level filter
       stateAndCov = EKF.runFilter(accMeas,gyroMeas-gyroBiasEst,attitude,otherMeas,dT,True)

#      print 'vel: ',stateAndCov[0][3:6].T
       #AttitudeFilt.updateYaw(stateAttitude)

       # publish
       #stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([stateAttitude.q]).T])
       stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([(attitude).q]).T, gyroMeas]).T
#       print "stateToPublish" , stateToPublish
       publishEstimatedState(currentTime,stateToPublish)
    
       # Update plot
       if (0):
           jj = jj+1
           if( jj == 30):

             '''biasPlotX.append(stateAndCov[0][12])
             biasPlotY.append(stateAndCov[0][13])
             biasPlotZ.append(stateAndCov[0][14])'''
             '''biasPlotX.append(EKF.qReference.q[0])
             biasPlotY.append(EKF.qReference.q[1])
             biasPlotZ.append(EKF.qReference.q[2])
             biasPlotW.append(EKF.qReference.q[3])'''
             biasPlotX.append(stateAndCov[0][9,0])
             biasPlotY.append(stateAndCov[0][10,0])
             biasPlotZ.append(stateAndCov[0][11,0])
             #biasPlotW.append(EKF.qReference.q[3])
             ymax = max(max(max(biasPlotX), max(biasPlotY)),max(biasPlotZ))
             print ymax
             ymin = min(min(min(biasPlotX), min(biasPlotY)),min(biasPlotZ))
             plt.ylim([ymin,ymax])
             line1.set_ydata(biasPlotX)
             line2.set_ydata(biasPlotY)
             line3.set_ydata(biasPlotZ)
             plt.draw()
             jj = 0
       

