import numpy as np
import scipy as sc
from scipy import integrate
import math
import Motor
import Propeller
import AeroQuaternion as AQ

np.random.seed()

global airDensity
airDensity = 1.225

class Multirotor:

# Members
    # Static parameters
    mass = []
    Inertia = []
    invInertia = []
    v_Q_i = AQ.Quaternion()    # rotation matrix from intertial to body
    cD = []              # drag coefficient         
    gravity = np.array([[0,0,9.81]]).T    # gravity in inertial
    motorList = []
    externalForce = []
    externalMoment = []
    # State variables
    # state = [x \
    #          y  >-> Position of vehicle cm expressed in inertial frame
    #          z /  
    #          u \
    #          v  >-> velocity of vehicle cm in inertial, expressed in vehicle frame
    #          w /
    #          qx  \
    #          qy   >-> Attitude of vehicle in inertial, quaternion
    #          qz  /
    #          qw /
    #          p \
    #          q  >-> Angular velocity of body in inertial, expressed in body frame
    #          r /
    stateVector = np.zeros((1,13))
    stateVector[0,9] = 1. # unit quat
    # code overhead
    lastTime = -.01  

# Methods

    # constructor
    def __init__(self,
                 motorType = "Motor",
                 propType = "Propeller",
                 motorPositions = [np.array([[.19,-.19,0]]).T,np.array([[.19,.19,0]]).T,
                                   np.array([[-.19,.19,0]]).T,np.array([[-.19,-.19,0]]).T],
                 motorOrientations = [[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1] ],
                 motorDirections = [1, -1, 1, -1],
                 fuselageMass = .7,
                 fuselageInertia = np.array([[.023385, 0, 0],[0, .018751, 0],[0, 0, .030992]]),
                 dragCoefficient = .001,
                 ):
        self.mass = fuselageMass
        self.Inertia = fuselageInertia
        self.cD = dragCoefficient
        # Inertia accounting
        Ixx = 0
        Iyy = 0
        Izz = 0
        Ixy = 0
        Ixz = 0
        Iyz = 0
        for ii in range(len(motorPositions)):
            # create motors (doing it with string allows for different prop/motor types
            createString = ("self.motorList.append(Motor." 
                           + motorType 
                           + "(position=motorPositions[ii],direction = motorDirections[ii],m_Q_v=AQ.Quaternion(motorOrientations[ii]),prop=Propeller." 
                           + propType + "()))")
            #self.motorList.append(Motor.Motor(position=motorPositions[ii],
            #                                  v_R_m=motorOrientations[ii]))
            exec createString
            motorMass = self.motorList[ii].mass
            propMass = self.motorList[ii].prop.mass
            # add mass of motor and props to total
            self.mass = self.mass + motorMass + propMass
            # motor/prop combos are treated as eccentric point masses for inertia calcs
            Ixx = Ixx + (motorMass+propMass)*(motorPositions[ii][0]**2)[0]
            Iyy = Iyy + (motorMass+propMass)*(motorPositions[ii][1]**2)[0]
            Izz = Izz + (motorMass+propMass)*(motorPositions[ii][2]**2)[0]
            Ixy = Ixy + (motorMass+propMass)*(motorPositions[ii][0]*motorPositions[ii][1])[0]
            Ixz = Ixz + (motorMass+propMass)*(motorPositions[ii][0]*motorPositions[ii][2])[0]
            Iyz = Iyz + (motorMass+propMass)*(motorPositions[ii][1]*motorPositions[ii][2])[0]
        # add effects of motor and prop intertias to the vehicle
        Imotors = np.array([[Ixx, -Ixy, -Ixz],[-Ixy, Iyy, -Iyz],[-Ixz, -Iyz, Izz]])
        self.Inertia = self.Inertia + Imotors  
        self.invInertia = np.linalg.inv(self.Inertia)
        print "inertia: ", self.Inertia, 'inverse inertia: ', self.invInertia


    def updateState(self,dT,motorCommands,windVelocity = np.zeros((3,1)),disturbance = 0,externalForces = []):
        # Initialize force and moment vectors
        Force = np.zeros((3,1))
        Moment = np.zeros((3,1))
        vel = np.array([self.stateVector[0,3:6]]).T
        omega = np.array([self.stateVector[0,10:]]).T
        self.v_Q_i = AQ.Quaternion(np.array(self.stateVector[0,6:10]))
        windVelBody = AQ.rotateVector(self.v_Q_i,windVelocity)
        state = 'flying'
        if (self.stateVector[0,2] >= 0 ): # on ground?
            if (np.dot( AQ.rotateVector(self.v_Q_i.inv(),vel).T ,np.array([[0,0,1]]).T ) > 2.): # falling fast
                state = 'ground'#'crashed'
            else:
                state = 'ground'
        #print state
        if (state == 'crashed'):
            return self.stateVector    
        # update all submodules (motors) and retrieve forces and moments
        for ii in range(len(self.motorList)):
            # update command
            self.motorList[ii].commandMotor(motorCommands[ii])
            # integrate state
            self.motorList[ii].updateState(dT,vehicleVelocity=vel,vehicleOmega=omega,windVelocity=windVelBody + disturbance*np.random.randn(3,1))
            # get aero forces and moments from motor
            FandM = self.motorList[ii].getForcesAndMoments()
            # Sum all forces
            Force = Force + FandM[0]
            # Sum all moments + r_motor X F_motor
            Moment = Moment + FandM[1] + np.cross(self.motorList[ii].position.T,FandM[0].T).T
        # add external forces, if any
        # each entry in externalForces must be a tuple, the first entry of which is the applied force, and the second of which is the application location.
        # both of these should be in body frame
        for exFor in externalForces:
            Force += exFor[0]
            Moment += np.cross(exFor[1].T,exFor[0].T).T
        # add drag force
        relWindVector = vel-windVelBody
        windMagnitude = np.sqrt(np.dot(relWindVector.T,relWindVector))
        eWind = relWindVector/(windMagnitude + .00000000001)
        dragForce = -0.5*airDensity*self.cD*windMagnitude**2*eWind
        Force = Force + dragForce  
        # add gravity
        Force = Force + self.mass*np.dot(self.v_Q_i.asRotMat,self.gravity)
        #Force = Force + self.mass*self.gravity
        # add ground force
        if (self.stateVector[0,2] >= 0):
            # Add normal force
            Kground = 1000
            Kdamp = 50
            Kangle = .001
            groundForceMag = Kground*self.stateVector[0,2] + Kdamp*np.dot(self.v_Q_i.asRotMat,vel)[2,0] 
            roll,pitch,yaw = self.v_Q_i.asEuler
            groundPitchMoment = max(min(Kangle*(-Kdamp*pitch - 6*Kdamp*self.stateVector[0,11]),1),-1) 
            groundRollMoment = max(min(Kangle*(-Kdamp*roll - 6*Kdamp*self.stateVector[0,10]),1),-1)
            groundYawMoment = Kangle*(-6*Kdamp*self.stateVector[0,12])
            Force = Force + AQ.rotateVector(self.v_Q_i,np.array([[0,0,-groundForceMag]]).T) - Kdamp*vel
            Moment = Moment + np.array([[0.,groundPitchMoment,0.]]).T + np.array([[groundRollMoment,0.,0.]]).T + np.array([[0.,0.,groundYawMoment]]).T
        self.externalForce = Force
        self.externalMoment = Moment
        #print state, self.mass*np.dot(self.v_Q_i.asRotMat,self.gravity).T
        # solve eoms
        #y = sc.integrate.odeint(self.vehicleEOM,self.stateVector[0,:],np.array([self.lastTime,self.lastTime + dT]))
        attitude = AQ.Quaternion(np.array(self.stateVector[0,6:10]))
        magOmega = np.linalg.norm(omega)
        axis = 1./(magOmega+1e-15)*omega
        updateQuat = AQ.quatFromAxisAngle(axis,dT*magOmega)
        self.stateVector = self.stateVector + dT*np.array([self.vehicleEOM(self.stateVector[0,:],0)])
        newAtt = updateQuat*attitude
        self.stateVector[0,6:10] = newAtt.q
        #self.stateVector[0,:] = y[-1][:]
        # Experiments with stupid Euler integration:
        #y = self.stateVector[0,:] + dT*self.vehicleEOM(self.stateVector[0,:],0)
        #self.stateVector[0,6:10] = AQ.normalize(self.stateVector[0,6:10])
        self.lastTime = self.lastTime + dT
        # update rotation matrix
        #print 'euler angles: ',self.v_Q_i.asEuler, 'rates: ',self.stateVector[0,10:]
        #print 'position: ', self.stateVector[0,0:3]
        measuredAcceleration = 1./self.mass*(self.externalForce  - self.mass*np.dot(self.v_Q_i.asRotMat,self.gravity))
        return [self.stateVector, measuredAcceleration]


    def vehicleEOM(self,y,t0):
        # Equations of motion for vehicle
        # Assumes that angular momentum from spinning rotors is negligible
        # Assumes that forces and moments are constant during a timestep
        # (should be reasonable for realistic timesteps, like 1ms)
        #
        # position derivatives equal rotated body velocities
        xDot = AQ.rotateVector(self.v_Q_i.inv(),y[3:6])
        # acceleration = F/m
        vDot = 1/self.mass*self.externalForce.T[0,:] - np.cross(np.array(y[10:]), np.array(y[3:6])).T
        # update quaternnions
        att = AQ.Quaternion(np.array(y[6:10]))
        #qDot = att.inv()*AQ.vectorAsQuat(.5*np.array([y[10:]]).T)
        qDot = AQ.qDotFromOmega(att,np.array(y[10:]))
        # omegadots
        effectiveMoment = self.externalMoment - np.array([np.cross( np.array(y[10:]), np.dot(self.Inertia,np.array(y[10:])))]).T
        omegadots = np.dot(self.invInertia,effectiveMoment).T[0,:]
        #print 'xdot: ',xDot.T[0]
        #print 'vdot: ',vDot
        #print 'qdot: ',qDot
        #print 'wdot: ',omegadots
        yDot = np.concatenate((xDot.T[0],vDot,qDot,omegadots),1)
        return yDot
