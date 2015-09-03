import numpy as np
import scipy as sc
from scipy import integrate
import math
import Propeller
import AeroQuaternion as AQ

class Motor:


# Members
  # Static Parameters
    # Motor Stuff
    prop = []      	# Attached propeller
    J = []         	# moment of inertia of rotor
    mass = []      	# "stuff"
    position = []  	# 3d vector of position of the motor, in vehicle coordinates
    #v_R_m = []     	# rotation matrix from motor to vehicle (probably I)
    m_Q_v = []
    direction = [] 	# +1 for CW, -1 for CCW as seen from above
    Kv = []             # Motor Kv (in units of radians/(sec*V) )
    R = []              # Coil resistance
    L = []              # Coil inductance
    Bmot = []           # Viscous damping
    # Speed controller
    tSC = .01           # time constant for lag 
    # state variables
    commandedValue = []	# command from controller
    localVelAirRel = []
    motorState = np.array([0,0])     # [i omega]
    torqueLoad = 0	# aero torque
    vBatt = 12.         # battery voltage
    # code overhead
    lastTime = -.01     # last time state was updated
    lastCmdTime = -.01  # last time command received
    




# Methods

    # constructor
    def __init__(self, prop = Propeller.Propeller(), J=.0000003, mass = .05, position = np.zeros((3,1)),
                 m_Q_v = AQ.Quaternion(), direction = 1, Kv = 1000, R = .1, L = .0003, Bmot = .00001 ):
        self.prop = prop
        self.J = J
        self.mass = mass
        self.position = position
        # self.v_R_m = v_R_m
        self.m_Q_v = m_Q_v
        self.direction = direction
        self.Kv = Kv*2*math.pi/60.   # Convert to rad/(s*V)
        self.R = R
        self.L = L
        self.Bmot = Bmot
        self.commandedValue = 0.
        self.motorState = np.array([0,0])

    def getForcesAndMoments(self):
    # vehicleVelocity and vehicleOmega should be I_v_bcm and I_w_veh, in vehicle coords.
    # windvelocity should have already been rotated into vehicle coords.     
    # returns forces and moments in vehicle reference frame
        localWind = -(self.localVelAirRel)
        omega = self.motorState[1]
        FandM = self.prop.calculateForcesAndMoments(omega,localWind,self.direction)
        output=[]
        output.append(AQ.rotateVector(self.m_Q_v.inv(),FandM[0]))
        output.append(AQ.rotateVector(self.m_Q_v.inv(),FandM[1]))
        return FandM

    def commandMotor(self, cmd = 0):
        # clip values out of range
        if(cmd > 1):
            cmd = 1
        elif (cmd < 0):
            cmd = 0
        self.commandedValue = cmd

    def commandAngle(self, ang = 0,axis = np.array([[1,0,0]]).T ):
        self.m_Q_v = AQ.quatFromAxisAngle(axis,ang)

    def updateState(self,dT,vehicleVelocity = np.zeros((3,1)),vehicleOmega = np.zeros((3,1)),
                    windVelocity = np.zeros((3,1))):
        # vehicle velocity, vehicle omega, and wind velocity are all given in vehicle frame
        # first, calculate local velocity relative to the air
        localVelAirRelVehFrame = vehicleVelocity + np.cross(vehicleOmega.T,self.position.T).T - windVelocity
        # rotate into motor frame (in case we decide to tilt motors
        self.localVelAirRel = AQ.rotateVector(self.m_Q_v,localVelAirRelVehFrame)
        # solve ODEs     
        #y = sc.integrate.odeint(self.motorEOM,self.motorState,np.array([self.lastTime, self.lastTime + dT]))
        #self.motorState = y[-1][:]
        # faster?
        #print 'motorState: ',self.motorState,'motorDeriv: ',self.motorEOM(self.motorState,dT)
        self.motorState = self.motorState + dT*self.motorEOM(self.motorState,dT)
        # then, calculate forces and moments
        self.lastTime = self.lastTime + dT;
        return self.motorState
   
    def motorEOM(self,y,t0):
        #  Equations of motion from Franklin, Powell & Emami. NB: Kv must be in rad/(s*V)
        #  L*didt + R*i = Va - 1/Kv*omega
        #  J*domegadt + b*omega = 1/Kv*i - Taero
        didt = 1./self.L*(self.vBatt*self.commandedValue - self.R*y[0] - 1/self.Kv*y[1] )
        # assume all rotational inertia comes from prop
        dwdt = 1./(self.prop.J + self.J)*(1/self.Kv*y[0] - self.Bmot*y[1] - 
                   self.prop.calcAxialAeroTorque(y[1],-self.localVelAirRel))
        dydt = np.array([didt,dwdt])
        return dydt

    def commandLag(self,dT):
        # filter command (just a little)
        alphaCmd = self.tSC/(self.tSC + dT)
        self.commandedValue = alphaCmd*self.commandedValue + (1.-alphaCmd)*cmd

    # Accessors
    def getAngularVelocity(self):
        return self.motorState[1]

    def getCurrent(self):
        return self.motorState[0]
