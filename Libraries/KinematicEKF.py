import numpy as np
import AeroQuaternion as AQ
import math 
grav = 9.81 
gravity = np.array([[0.],[0.],[grav]])
gravdir = np.array([[0.,0.,1.]]).T
def firstOrdLowpass(val,meas,freq,dt):
    tau = 1./freq
    alf = tau/(tau+dt)
    #print val, alf, meas
    filtval = alf*val + (1.-alf)*meas
    return filtval


def crossMat(vec):
    # create skew-symmetric cross product matrix
    CrossMat = np.zeros((3,3))
    if (vec.shape == (3,1)):
       x,y,z = vec.T[0]
    elif (vec.shape == (1,3)):
       x,y,z = vec[0]
    CrossMat[0,1] = -z
    CrossMat[0,2] = y
    CrossMat[1,0] = z
    CrossMat[1,2] = -x
    CrossMat[2,0] = -y
    CrossMat[2,1] = x
    return CrossMat


class kinematicQuadEKF:

    ''' 
       This class runs a multiplicative kinematic extended kalman filter
       It is driven by attitude, angular velocity, and acceleration measurements.
       Estimated parameters:
          - 3D Position in inertial frame 
          - 3D Velocity in inertial frame
          - mounting error of the imu (only pitch and roll)
          - accelerometer bias

       The mounting error is stored in self.qReference

       state = [x y z u v w gibbsVectorOfMountingError accBias]'

       for more information on the inner workings of the MEKF, search for papers written by Landis Markley out of NASA

    '''

    # constructor

    def __init__(self,initialState = np.array([[0,0,0,0,0,0,0,0,0,0,0,0]],dtype=np.float64).T,initialCovariance = 1*np.eye(12,dtype=np.float64)):

        self.state = initialState
        self.cov = initialCovariance
        self.gravEstimate = -gravity    
        self.qReference = AQ.Quaternion() # this is now a delta reference off the reported attitude

    def runFilter(self,accMeas,gyroMeas,attitude,otherMeas,dT,flyingFlag=True):
        '''
            gyro and accel measurements are 3x1 np.arrays
            dt is a scalar
            otherMeas is a list of tuples,
               each tuple has the format
               ['measurement name', measurement value, measurement covariance matrix]

            eg. 
               otherMeas = [['gps', [3x1 np array], [3x3 covariance]], ['barometer', height, sigmasquared ]
        '''
        # integrate EOMs to get predicted mean and covariance
        self._predictStep(accMeas,gyroMeas,attitude,dT,flyingFlag)
        # update with other measurements
        #if (len(otherMeas) > 0):
        self._updateStep(attitude,otherMeas)
        # update reference quaternion
        self._resetQuat()
        #self.gyroBias = self.gyroBias + biasDot*dT
        # all done 
        return [self.state, self.cov]


    def _predictStep(self,accMeas,gyroMeas,attitudeReported,dT,flyingFlag=True):
        ########################################
        ## Make things pretty for readability ##
        ########################################
        u = np.vstack([accMeas,gyroMeas])
        position = self.state[0:3].copy()
        velocity = self.state[3:6].copy()
        accBias = self.state[9:12].copy()
        # Attitude is quaternion product of mounting error estimate and reported attitude
        attitude = self.qReference*attitudeReported
        ########################################
        ## Propagate dynamics                 ##
        ########################################
        # nonlinear state estimate update derivatives
        xDot = np.dot(attitude.asRotMat.T,velocity)
        velDot = (accMeas-accBias) + np.dot(attitude.asRotMat,gravity) - np.dot(crossMat(gyroMeas),velocity)
        # debug printouts
        if (False): 
            print 'acc Meas: ', accMeas.T
            print 'velocity dot: ',velDot.T
            print 'velocity: ',velocity.T
            print 'mountingError: ',self.qReference.asEuler
            print 'reported att',attitudeReported.asEuler
            print 'acc bias: ',accBias.T
        # update state via Euler integration
        self.state[0:3] = position + xDot*dT
        self.state[3:6] = velocity + velDot*dT
        self.state[6:9] = np.zeros((3,1))
        # state update is now taken care of. move on to covariance
        ########################################
        ## Populate covariance matrix (Amat)  ##
        ########################################
        # linearize EOMs for covariance update
        Amat = np.zeros((self.state.shape[0],self.state.shape[0]))
        # Populate Amat
        # Amat [0:3,0:3] is the partial derivative of position with respect to position, so the identity matrix
        Amat[0:3,0:3] = np.eye(3)
        # Amat[0:3,3:6] is the partial derivative of position with respect to velocity. Since we are using body-components of velocity
        # this is i_R_b*dt, where i_R_b is the rotation matrix such that --> velocityInInertial = i_R_b*velocityInBody
        # the (dT/.1) is a heuristic that accounts for the fact that we're not receiving a position update for every integration update
        # I think what's going on is linearization-related, but lessening the dependence between velocity and position estimates like this
        # seems to work pretty well. Stated another way, this term is what ties position and velocity estimates together. If it looks like 
        # there's too much coupling between those two, reduce this term.
        #Amat[0:3,3:6] = (dT/.1)*(attitude.asRotMat.T*dT)
        Amat[0:3,3:6] = attitude.asRotMat.T*dT # <- no fudge factor here. Straight linearization
        # Right now we only estimate mounting error if we're flying, because the estimator has a tough time telling accelerometer bias
        # from mounting error unless it's free to move... I think
        # for information on how this estimate works, see "Vision-aided inertial navigation for pin-point landing using observations of mapped landmarks" by
        # Nikolas Trawny et. al.
        if(flyingFlag):
           #Amat[0:3,6:9] = -crossMat(np.dot(attitude.asRotMat.T,velocity))*dT
           Amat[0:3,6:9] = -np.dot(attitudeReported.asRotMat.T,crossMat(velocity))*dT
        # Covariance of velocity with itself. Because it's being recorded in the inertial frame, it has a cross product term with the gyro
        Amat[3:6,3:6] = np.eye(3) - crossMat(gyroMeas)*dT
        # Tying mounting error to velocity
        if(flyingFlag):
           Amat[3:6,6:9] = crossMat(np.dot(attitudeReported.asRotMat,gravity))*dT
        # Describing velocity's dependence on estimated accelerometer bias
        Amat[3:6,9:12] = -dT*np.eye(3)
        # Covariance of mounting error with itself. Again, see papers by Trawny and Markley for more details
        Amat[6:9,6:9] = np.eye(3) - crossMat(gyroMeas)*dT
        # Covariance of acc bias with itself. Assumed that bias doesn't change (much) with time.
        Amat[9:12,9:12] = np.eye(3)
        # update covariance
        ########################################
        ## Process noise matrix!!             ##
        ########################################
        ##########################################################
        ##   _    _    _                        _    _    _     ##
        ##  / \  / \  / \    Twiddle these     / \  / \  / \    ##
        ##  \_/  \_/  \_/       KNOBS!         \_/  \_/  \_/    ##
        ##                                                      ##
        ##########################################################
        # This is the single largest fudge-factor in the update step, and your main knob to tweak. This roughly corresponds to how confident you are in your 
        # motion model, but since we're using a kinematic model (so we know the equations of motion exactly) it's really more an indicator of how much we trust
        # our attitude estimate, accelerometers and gyroscopes, which we're using to drive the filter. 
        RT = 1e-6*np.diag(np.array([.1,.1,.1,.01,.01,.01,.00001,.00001,.00001,100,100,100]),0)
        ########################################
        ## Update the state covariance matrix ##
        ########################################
        self.cov = np.dot(Amat,np.dot(self.cov,Amat.T)) + RT
        #------------ End of predict step -----------------

    def _updateStep(self,attitudeReported,Measurements):
        # Current attitude estimate
        attitude = self.qReference*attitudeReported
        # Do we have any new low-frequency measurements?
        if (len(Measurements) == 0):
          return
        # If yes, unpack them:
        ########################################
        ## Unpack Low-frequency measurements  ##
        ########################################
        for ii in range(0,len(Measurements)): # loop over all measurements
           if(Measurements[ii][0] == 'gps'):
                # reports x,y,z !!! make sure frame is correct
                # print 'gps:',Measurements[ii][1]
                Cii = np.hstack([np.eye(3), np.zeros((3,9))]) 
                measii = Measurements[ii][1]
                Qii = Measurements[ii][2]
                expectedMeas = np.dot(Cii,self.state)
           elif(Measurements[ii][0] == 'barometer'):
                # reports only z 
                Cii = np.zeros((1,12))
                Cii[0,2] = 1          
                measii = Measurements[ii][1]       
                Qii = Measurements[ii][2]
                expectedMeas = np.dot(Cii,self.state)
           # KF isn't handling yaw measurements now
           elif(False):#Measurements[ii][0] == 'mag'): # Interpret this as yaw measurement
                print 'mag!'
                expectedMeas = np.dot(attitude.asRotMat,np.array([[1],[0],[0]]))
                print 'exp, mag: ', expectedMeas
                print 'meas, mag: ', Measurements[ii][1] 
                Cii = np.hstack([np.zeros((3,6)), crossMat(expectedMeas),np.zeros((3,3))]) 
                measii = Measurements[ii][1]       
                Qii = Measurements[ii][2]
           # now concatenate measurement vectors and matrices
           if (ii > 0):
              C = np.vstack([C, Cii])
              measurementVec = np.vstack([measurementVec, measii])
              expectedMeasVec = np.vstack([expectedMeasVec, expectedMeas])
              sensorNoise = np.vstack((np.hstack((sensorNoise,np.zeros((sensorNoise.shape[0],Qii.shape[1])))),
                                       np.hstack((np.zeros((Qii.shape[0],sensorNoise.shape[1])),Qii))))
           else:
              # Initialize C-matrix
              C = Cii
              measurementVec = measii
              expectedMeasVec = expectedMeas
              sensorNoise = Qii
        ########################################
        ## Do Kalman Math                     ##
        ########################################
        # finally perform measurement update
        # Schur complement of covariance (makes next steps cleaner
        schur = np.dot(   np.dot(C,self.cov) ,   C.T) + sensorNoise
        # Calculate Kalman gain
        Kgain = np.dot(   np.dot(self.cov, C.T), np.linalg.inv(schur) )
        # Update state with Kalman update
        self.state = self.state + np.dot(Kgain , measurementVec - expectedMeasVec)
        # Update covariance with Kalman update
        self.cov = self.cov - np.dot(Kgain,np.dot(C,self.cov))
        if (False): # DEBUG
           print 'exp meas'
           print expectedMeasVec
           print 'meas'
           print measurementVec
           print 'meas err'
           print measurementVec - expectedMeasVec
           print 'K*err'
           print np.dot(Kgain, measurementVec - expectedMeasVec)
        #------------ End of update step -----------------

    def _resetQuat(self):
        # In this step we push any mounting error calculated by the measurement update step
        # onto the (non-stochastic) reference quaternion carried by the filter.
        # Unpack Gibbs vector from state
        errorAngle = np.linalg.norm(self.state[6:9])
        # Gibbs vector is an axis-angle representation of mounting error. Turn this into a quaternion
        qReset = AQ.quatFromAxisAngle(self.state[6:9]/(errorAngle + 1e-13),errorAngle)
        # Push error onto reference quaternion
        self.qReference = qReset*self.qReference
        # We don't want to estimate a heading offset (seemed to be getting us into trouble)
        eulers = self.qReference.asEuler
        # set yaw to zero
        eulers[2] = 0.
        # reconstitute reference quaternion
        self.qReference = AQ.Quaternion(eulers)
        #self.qReference = AQ.Quaternion() # turn off mount error estimation
        # zero Gibbs vector
        self.state[6:9] = np.zeros((3,1))


class AttitudeComplementaryFilter:

    def __init__(self,initialAttitude = AQ.Quaternion(), timeConstant = 15., Kbias = 0.000):
        '''
        timeConstant is (roughly) the time it takes the filter to trust the accelerometer 100%
        e.g. if you initialize the quad on an incline, the gyros will read zeros. With the default 
        timeConstant value of 10, it would take about ten seconds for the estimate of attitude to 
        converge to the true (tilted) attitude.
  
        Kbias controls how quickly the filter learns a gyro bias. Too big and your quad's estimate will swing all over the place like
        a drunkard. Too little and it won't learn any real bias that might exist. 0.01 seemed to work alright on real data.

        '''
        self.attitudeEstimate = initialAttitude
        self.gyroBias = np.zeros((3,1));
        self.Kbias = Kbias
        self.timeConstant = .1
        self.tcTarget = timeConstant

    def runFilter(self,accMeas,gyroMeas,otherMeas,dT):

        #spool up after initialization
        if (self.timeConstant < self.tcTarget) :
 	   self.timeConstant += .00001*(self.tcTarget - self.timeConstant) + .001
        #self.timeConstant = self.tcTarget
        # integrate angular velocity to get 
        omega = gyroMeas - self.gyroBias;
        # integrate attitude
        attHat = self.gyroUpdate(self.attitudeEstimate,omega,dT)
        # accel Update
        newAtt = self.accelUpdate(attHat,accMeas,dT/(dT+self.timeConstant),accThresh=.2)
        # mag update
        for meas in otherMeas:
           if (meas[0] == 'mag'):
             magMeas = meas[1]
             earthMag = meas[2]
             newAtt = self.magUpdate(newAtt,magMeas,dT/(dT+self.timeConstant),earthMag)
        # calculate difference between measurement and integrated for bias calc
        errorQuat = (newAtt*attHat.inv()).q
        # calculate bias derivative
        biasDot = -np.dot(np.array([[self.Kbias,0,0],[0, self.Kbias,0],[0,0,self.Kbias]]),(1./errorQuat[3])*np.array([[errorQuat[0]],[errorQuat[1]],[errorQuat[2]]]))
        # Update gyro bias
        self.gyroBias = self.gyroBias + dT*biasDot
        # update attitude
        newAtt.normalize()
        self.attitudeEstimate = newAtt

             
        return [self.attitudeEstimate, self.gyroBias]

    def gyroUpdate(self,att,omega,dT):
        dQ = quatFromRotVec(omega*dT)
        return dQ*att

    def accelUpdate(self,attHat,aMeas,alpha, accThresh = .1):
        if (np.abs(np.linalg.norm(aMeas) - 9.81) > accThresh):
          return attHat
        # if measuring 1 g
        aMeasNorm = aMeas.T/np.linalg.norm(aMeas)
        gBody = np.dot(attHat.asRotMat,-gravdir)
        axisangle = np.cross(aMeasNorm,gBody.T).T
        angle = np.arcsin(np.linalg.norm(axisangle))
        axis = (1./(np.linalg.norm(axisangle)+1e-5))*axisangle
        #qAcc = quatFromRotVec(alpha*axisangle)
        qAcc = AQ.quatFromAxisAngle(axis,alpha*angle)
        # rotate between integrated and measured attitude
        return qAcc*attHat

    def magUpdate(self,attHat,magMeas,alpha,worldMag):
        '''
        expectedMag = np.dot(attHat.asRotMat,worldMag)
        axisangle = np.cross(magMeas.T,expectedMag.T).T
        axis = 1./((np.linalg.norm(axisangle))+1e-5)*axisangle
        angle = np.arcsin(np.linalg.norm(axisangle))
        #print axis.T, angle
        #print ' '
        #qMag = quatFromRotVec(alpha*axisangle)
        qMag = AQ.quatFromAxisAngle(axis,alpha*angle)
        '''
        # Code for using mag ONLY for yaw
        # rotate mag reading into world
        magInWorld = np.dot(attHat.asRotMat.T,magMeas)
        magInWorld[2][0] = 0.0
        magInWorld = 1/(np.linalg.norm(magInWorld))*magInWorld
        worldMag[2][0] = 0.0
  	worldMag = 1/(np.linalg.norm(worldMag))*worldMag
        axisSinAngle = np.cross(magInWorld.T,worldMag.T).T
        angle = np.arcsin(np.linalg.norm(axisSinAngle))
        axis = (1./(np.linalg.norm(axisSinAngle) + 1e-5))*axisSinAngle
        print axis.T,angle
        #axis = np.dot(attHat.asRotMat,axis)
        qMag = AQ.quatFromAxisAngle(axis,angle*alpha)
        return attHat*qMag #qMag*attHat #AQ.slerp(attHat,qMag,alpha)
    


def attFromAccel(yawInDeg,acc):
    roll = math.atan2(-acc[1,0],-acc[2,0])
    pitch = math.asin(acc[0,0]/(np.linalg.norm(acc) + 1E-12))
    return AQ.Quaternion(np.array([180./np.pi*roll,180./np.pi*pitch,yawInDeg])) # constructor takes eulers in degrees


def quatFromRotVec(vec):
    angle = np.linalg.norm(vec)
    if (angle == 0.):
      return AQ.Quaternion([0.,0.,0.,1.])
    axis = vec/angle
    return AQ.quatFromAxisAngle(axis,angle)

###############################################################
## The following code is not being used for anything         ##
###############################################################


class PositionComplementaryFilter:

    def __init__(self,timeConstant = 10., Kbias = 0.5):

        self.velocityEstimate = np.zeros((3,1))
        self.positionEstimate = np.zeros((3,1))
        self.positionCorrection = np.zeros((3,1))
        self.positionBase = np.zeros((3,1))
        self.accCorrection = np.zeros((3,1))
        self.diff = np.zeros((3,1));
        self.diffInt = np.zeros((3,1))
        self.Kbias = Kbias
        self.Kacc = .5
        self.Kvel = 50
        self.Kpos = 1
        self.timeConstant = timeConstant

    def runFilter(self,accMeas,otherMeas,dT):
 	
        # accMeas must be rotated into earth frame
        # integrate angular velocity to get 
        positionError = np.zeros((3,1))
        for iMeas in otherMeas:
           if (iMeas[0] == 'gps'):
              positionError = iMeas[1] - (self.positionEstimate + self.positionCorrection)
        # now integrate stuff
        self.accCorrection += positionError*dT*self.Kacc
        self.velocityEstimate  += positionError*dT*self.Kvel
        self.positionCorrection += positionError*dT*self.Kpos
        velIncrease = ((accMeas+gravity) + self.accCorrection)*dT
        self.positionBase += (self.velocityEstimate + velIncrease*.5)*dT
        self.positionEstimate = self.positionBase + self.positionCorrection
        self.velocityEstimate += velIncrease    
        return [self.positionEstimate, self.velocityEstimate, self.accCorrection]



