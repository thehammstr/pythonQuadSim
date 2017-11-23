import numpy as np
import scipy as sc
from scipy import integrate
import math
import Motor
import Propeller
import AeroQuaternion as AQ
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
np.random.seed()


class InvertedPendulum:

# Members
    # Static parameters
    mass = []
    Inertia = []
    invInertia = []
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    wheel_radius = .05
    height = .3
    depth = .1
    wheelbase = .30
    cg = .2
    gravity = np.array([[0,0,9.81]]).T    # gravity in inertial
    # State variables
    # state = [x \
    #          y  >-> Position of vehicle cm expressed in inertial frame
    #          z /
    #          vx >-> velocity of vehicle axle in inertial, expressed in yawed frame
    #          vy
    #          vz
    #          roll
    #          pitch
    #          yaw
    #          roll rate p
    #          pitch rate q
    #          yaw rate r
    stateVector = np.zeros((1,12))
    stateVector[0,2] = -wheel_radius # unit quat
    # code overhead
    lastTime = -.01

# Methods

    # constructor
    def __init__(self,
                 Mass = 2.0,
                 Inertia = np.array([[.023385, 0, 0],[0, .018751, 0],[0, 0, .030992]]),
                 ):
        print "Init called"
        self.mass = Mass
        self.Inertia = Inertia

    def updateState(self,dT,motorCommands):
        # Initialize force and moment vectors
        Force = np.zeros((3,1))
        Moment = np.zeros((3,1))
        pos = np.array([self.position()]).T
        vel = np.array([self.stateVector[0,3:6]]).T
        omega = np.array([self.stateVector[0,10:]]).T
        phi,theta,psi = self.attitude()
        self.v_Q_i = AQ.Quaternion([phi,theta,psi])
        # GROUND FORCES
        # wheel forces and top forces if fallen (ASSUMES FLAT GROUND)
        # left_wheel_contact = self.pos + i_R_v*[0, -wheelbase/2,0] + [0,0,radius]
        i_R_v = self.v_Q_i.asRotMat.T
        cg_pos = pos + np.dot(i_R_v,np.array([[0, 0, -self.cg]]).T)
        left_wheel_contact = pos + np.dot(i_R_v,np.array([[0,-self.wheelbase/2.0,0]]).T) + np.array([[0.0,0.0,self.wheel_radius]]).T
        right_wheel_contact= pos + np.dot(i_R_v,np.array([[0, self.wheelbase/2.0,0]]).T) + np.array([[0.0,0.0,self.wheel_radius]]).T
        top_contact = pos + np.dot(i_R_v,np.array([[0, 0, -self.height]]).T) + np.array([[0.0,0.0,self.depth]]).T
        contactPoints = [left_wheel_contact,right_wheel_contact,top_contact]
        GroundForce,GroundMoment = self.groundForces(contactPoints,cg_pos,[])
        self.externalForce = GroundForce
        self.externalMoment = GroundMoment
        # GRAVITY
        self.externalForce = self.externalForce + self.mass * self.gravity
        #print state, self.mass*np.dot(self.v_Q_i.asRotMat,self.gravity).T
        # solve eoms
        #y = sc.integrate.odeint(self.vehicleEOM,self.stateVector[0,:],np.array([self.lastTime,self.lastTime + dT]))
        attitude = AQ.Quaternion(np.array(self.stateVector[0,6:10]))
        magOmega = np.linalg.norm(omega)
        axis = 1./(magOmega+1e-15)*omega
        updateQuat = AQ.quatFromAxisAngle(axis,dT*magOmega)
        #self.stateVector = self.stateVector + dT*np.array([self.vehicleEOM(self.stateVector[0,:],0)])
        #newAtt = updateQuat*attitude
        #self.stateVector[0,6:10] = newAtt.q
        #self.stateVector[0,:] = y[-1][:]
        # Experiments with stupid Euler integration:
        #y = self.stateVector[0,:] + dT*self.vehicleEOM(self.stateVector[0,:],0)
        #self.stateVector[0,6:10] = AQ.normalize(self.stateVector[0,6:10])
        #self.lastTime = self.lastTime + dT
        # update rotation matrix
        #print 'euler angles: ',self.v_Q_i.asEuler, 'rates: ',self.stateVector[0,10:]
        #print 'position: ', self.stateVector[0,0:3]
        #measuredAcceleration = 1./self.mass*(self.externalForce  - self.mass*np.dot(self.v_Q_i.asRotMat,self.gravity))
        return [self.stateVector,np.zeros((3,1))]


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
        yDot = np.concatenate((xDot.T[0],vDot,qDot,omegadots))
        return yDot

    def groundForces(self,contactPoints, cg, Map = []):
        Force = np.zeros((3,1))
        Moment = np.zeros((3,1))
        for point in contactPoints:
            if (point[2,0] >= 0): # TODO use map
                # Add normal force
                Kground = 1000
                Kdamp = 50 # TODO: need damping?
                Kangle = .001
                groundForceMag = Kground*point[2,0]
                groundForce = np.array([[0,0,-groundForceMag]]).T
                Force = Force + groundForce
                Moment = Moment + np.cross((point-cg).T,groundForce.T).T
        return [Force,Moment]

    def attitude(self):
        return self.stateVector[0,6:9]
    def position(self):
        return self.stateVector[0,0:3]

    def draw(self,color = [1,0,0,1],wheel_color = [0,1,0,1]):
        glPushMatrix()
        glTranslate(self.stateVector[0,0],self.stateVector[0,1],self.stateVector[0,2])
        glPushMatrix()
        glRotate(180.0/np.pi*self.attitude()[2],0,0,1)
        glRotate(180.0/np.pi*self.attitude()[1],0,1,0)
        glRotate(180.0/np.pi*self.attitude()[0],1,0,0)
        # Body
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, color)
        q = gluNewQuadric()
        glPushMatrix()
        glRotate(90.,1.0,0.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.wheelbase/2.,32,32);
        glPopMatrix()
        glPushMatrix()
        glRotate(-90.,1.0,0.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.wheelbase/2.,32,32);
        glPopMatrix()
        glPushMatrix()
        glRotate(180.,0.0,1.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.height,32,32);
        glPopMatrix()
        glPushMatrix()
        glTranslate(0,0,-self.height)
        glRotate(180.,0.0,1.0,0.0)
        gluCylinder(q,self.depth/2,self.depth,self.depth,32,32);
        glPopMatrix()
        # Left wheel
        glPushMatrix()
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, wheel_color)
        glTranslate(0.0,-self.wheelbase/2,0.0)
        glPushMatrix()
        glRotate(90.0,1.0,0.0,0.0)
        glutSolidTorus(0.01,self.wheel_radius,20,20)
        glPopMatrix()
        glPopMatrix()
        # Right wheel
        glPushMatrix()
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, wheel_color)
        glTranslate(0.0,self.wheelbase/2,0.0)
        glRotate(90.0,1.0,0.0,0.0)
        glutSolidTorus(0.01,self.wheel_radius,20,20)
        glPopMatrix()

        glPopMatrix() # body rotation
        glPopMatrix() # body translation

