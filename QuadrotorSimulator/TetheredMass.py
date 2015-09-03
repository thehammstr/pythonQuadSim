import numpy as np
import scipy as sc
import math
class TetheredMass:


# Static Parameters

# Methods

    # constructor
    def __init__(self, mass = .5, tetherLength = 5.5, stiffness = 80, initialState = np.zeros((6,1))):
        self.mass = mass
        self.stiffness = stiffness
        self.tetherLength = tetherLength
        self.state = initialState
        self.lastStretch = 0
        return

    def update(self, supportLocation, dT):

       position = self.state[0:3].copy()
       velocity = self.state[3:6].copy()
       # calculate force
       vec = supportLocation - position
       normVec = np.linalg.norm(vec)
       vecHat = (1./(normVec + 1e-15))*vec
       extension = normVec - self.tetherLength
       
       # add gravity
       Force = self.mass*np.array([[0],[0],[9.81]])
       # if under tension
       if (extension >= 0):
           Fmag = self.stiffness*extension 
           #Force += Fmag*vecHat - 5*np.dot(velocity.T,vecHat)*vecHat 
           Force += Fmag*vecHat + 350*((extension - self.lastStretch)**1)*vecHat
       # Slight absolute damping
       Force -= .001*velocity
       # ground force
       if (position[2,0] >= 0):
            # Add normal force
            Kground = 1000
            Kdamp = 50
            groundForceMag = Kground*position[2,0] 
            Force += np.array([[0],[0],[-groundForceMag]]) - Kdamp*velocity
       # now update state
       pos = position + velocity*dT
       vel = velocity + (1/self.mass)*Force*dT
       self.state[0,0] = pos[0,0] 
       self.state[1,0] = pos[1,0]
       self.state[2,0] = pos[2,0]
       self.state[3,0] = vel[0,0]
       self.state[4,0] = vel[1,0]
       self.state[5,0] = vel[2,0] 
       self.lastStretch = extension
       if(extension >= 0):
           return - ( Fmag*vecHat + 350*((extension - self.lastStretch)**1)*vecHat )
       else:
           return np.zeros((3,1))
           
