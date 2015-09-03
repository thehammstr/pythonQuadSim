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

#############################################################
#############################################################
#
#    USER-DEFINED parameters useful to simulation
#
#
#
#
#
#############################################################
#############################################################

# Constant gyro bias?
gyro_bias = np.array([[0.],[0.],[0.]])
# mounting error of imu on quad, in degrees
mount_error_roll = 0. 
mount_error_pitch = 0. 
mount_error_yaw = 0. 
# Wind
windvel = np.zeros((3,1))
# standard deviation of disturbance force due to turbulence
disturbance = 10.
# time step
dt = 0.005

#############################################################
#############################################################
#
#    END USER-DEFINED parameters
#
#############################################################
#############################################################

# ZMQ shit
# Import ZMQ, our fancy-pants networking thingy
import zmq

# Import our custom "state_pb2" protobuf-python code
from protobuf.py import state_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

ControlReceiver = context.socket(zmq.SUB)
ControlReceiver.bind("tcp://*:5003")
ControlReceiver.setsockopt(zmq.SUBSCRIBE, "")

# Create our Publishing socket here
socketPose = context.socket(zmq.PUB)
socketEKF = context.socket(zmq.PUB)
# Bind our publishing socket to localhost on some port
socketPose.bind("tcp://*:5000")
socketEKF.bind("tcp://*:5001")



#############################################################
############# Paul, this is where you need to put ###########
############# your socket poller stuff ######################

#poller = zmq.Poller()
#poller.register(IMUReceiver, zmq.POLLIN)


#####################
def publishState(time,state,socket):
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

        socket.send( pose.SerializeToString() )


def processControlData(data):
     # Fill this in based on how controls are passed
     commands = [data.roll, data.pitch, data.yaw_rate, data.throttle]
     return commands

# create objects
Quad = Multirotor.Multirotor() # default is quadrotor
controller = QuadrotorController.Controller()
AttEstimator = KinematicEKF.AttitudeComplementaryFilter()
EKF = KinematicEKF.kinematicQuadEKF()


Quad.stateVector[0,2] = 0. # initial height
np.random.seed([])
gravTracker = 0.
accMeas = np.zeros((3,1))
lastUpdateTime = 0.
mountError = AQ.Quaternion(np.array([mount_error_roll,mount_error_pitch ,mount_error_yaw]))

# Set up ZMQ subscriber/poller stuff
poller = zmq.Poller()
poller.register(ControlReceiver, zmq.POLLIN)
Controldata = state_pb2.AngleCommandPb()

motor_commands = [0.,0.,0.,0.]

lastUpdateTime = 0.
commands=[0,0,0,0]
debug_print_count=0
while(True):
    # Note start time
    startTime = clock.time()
    # Run simulator
    state,acc = Quad.updateState(dt,motor_commands,windVelocity = windvel,disturbance = disturbance)
    accMeas = acc + .001*np.array([np.random.randn(3)]).T + np.array([[.0],[0],[0]])
    # Get gyro Meas
    gyroMeas = state.T[10:] + .001*np.array([np.random.randn(3)]).T + np.array([[0.0],[0],[0]]) #+ np.array([[0.],[0.01],[.0]])# + .001*np.ones((3,1))
    posMeas = state.T[0:3] + .01*np.array([np.random.randn(3)]).T
    attTrue = AQ.Quaternion(state[0,6:10])
    magMeas = np.dot(attTrue.asRotMat,np.array([[1.,0.,0.]]).T)
    if (startTime-lastUpdateTime > .2):
        otherMeas = []
        otherMeas.append(['gps',posMeas,.01*np.eye(3)])
        otherMeas.append(['mag',magMeas,.1*np.eye(3)])
        otherMeas.append(['barometer',posMeas[2,0]+.01*np.random.rand(),2*np.ones((1,1))])
        lastUpdateTime = startTime
    else:
        otherMeas = []
    # run attitude filter
    attitude,gyroBiasEst = AttEstimator.runFilter(accMeas,gyroMeas,otherMeas,dt)
    # run high level filter
    stateAndCov = EKF.runFilter(accMeas,gyroMeas,attitude,otherMeas,dt,True)

    # Publish State
    publishState(startTime,state,socketPose)
    stateAttitude = EKF.qReference
#    stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([(stateAttitude*mountError*attTrue).q]).T, gyroMeas-gyroBiasEst]).T
    stateToPublish_simpleatt = np.vstack([stateAndCov[0][0:6], np.array([(attitude).q]).T, gyroMeas]).T
#    print 'stateToPublish', stateToPublish
#    print 'stateToPublish2', stateToPublish2
    publishState(startTime,stateToPublish_simpleatt,socketEKF)

    # read in data from zmq
    try:
        data = ControlReceiver.recv(flags=zmq.NOBLOCK)
        Controldata.ParseFromString(data)
        commands = processControlData(Controldata)
    except:
        'NO ANGLE COMMAND'
        #print
    if (1):
        debug_print_count += 1
        if (debug_print_count%200==0):
            print 'est. mount err: ', EKF.qReference.asEuler

    # run controller
    #stateToPublish = np.vstack([stateAndCov[0][0:6], np.array([(stateAttitude*mountError*attTrue).q]).T])
    #controlState = np.vstack([stateToPublish,gyroMeas-attitudeAndGyroBias[1]])
    #motor_commands = controller.updateControl(dt,controlState.T,commands,'xyah')
    motor_commands = controller.updateControl(dt,stateToPublish_simpleatt,commands,'rpYRt')



    # make it realtime
    while(clock.time() - startTime < dt):
       pass


