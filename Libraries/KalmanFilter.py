import numpy as np
import AeroQuaternion as AQ
import math
#import matplotlib.pyplot as plt
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

#
# Position Jacobians
#

def dfxdx():
  # Jacobian of f(x) wrt x (position)
  return np.eye(3)

def dfxdv(dT):
  # Jacobian of f(x) wrt velocity
  return dT*np.eye(3)

def dfxdq(dT,quat,accMeasBody):
  # Jacobian of f(x) wrt attitude)
  qx,qy,qz,qw = quat.q
  # get partial wrt each quat component
  d_dqx = 0.5*dT*dT*np.dot(drdq(0,qx,qy,qz,qw),accMeasBody)
  d_dqy = 0.5*dT*dT*np.dot(drdq(1,qx,qy,qz,qw),accMeasBody)
  d_dqz = 0.5*dT*dT*np.dot(drdq(2,qx,qy,qz,qw),accMeasBody)
  d_dqw = 0.5*dT*dT*np.dot(drdq(3,qx,qy,qz,qw),accMeasBody)


  return np.hstack((d_dqx,d_dqy,d_dqz,d_dqw))

def dfxda(dT,quat):
  # Jacobian of position wrt accel
  return 0.5*dT*dT*quat.asRotMat.T

def dfxdaB(dT,quat):
  # accel bias position jacobian
  return -dfxda(dT,quat)

#
# Velocity Jacobians
#

def dfvdv():
  return np.eye(3)

def dfvdq(dT,quat,accMeasBody):
  # Jacobian of f(v) wrt attitude)
  qx,qy,qz,qw = quat.q
  # get partial wrt each quat component
  d_dqx = dT*np.dot(drdq(0,qx,qy,qz,qw),accMeasBody)
  d_dqy = dT*np.dot(drdq(1,qx,qy,qz,qw),accMeasBody)
  d_dqz = dT*np.dot(drdq(2,qx,qy,qz,qw),accMeasBody)
  d_dqw = dT*np.dot(drdq(3,qx,qy,qz,qw),accMeasBody)

  return np.hstack((d_dqx,d_dqy,d_dqz,d_dqw))

def dfvda(dT,quat):
  return dT*quat.asRotMat.T

def dfvdaB(dT,quat):
  return -dfvda(dT,quat)

#
# Attitude Jacobians
#

def dfqdq(dT,ang_vel):
  # Jacobian of attitude wrt itself
  omegaCross = crossMat(ang_vel)
  Mat1 = np.hstack((omegaCross,ang_vel))
  Mat2 = np.hstack((ang_vel.T, [[0]]))
  Mat = np.vstack((Mat1,Mat2))
  return np.eye(4)+0.5*dT*Mat

def dfqdw(dT,att):
  # Jacobian of attitude wrt angular velocity
  qx,qy,qz,qw = att.q
  vq = np.array([[qx,qy,qz]]).T
  return 0.5*dT*np.vstack((qw*np.eye(3) - crossMat(vq), -vq.T))

def dfqdwB(dT,att):
  # Jacobian of attitude wrt ang vel bias
  return -dfqdw(dT,att)


def drdq(i,qx,qy,qz,qw):

  # calculate the jacobian of Rotation matrix w_R_b wrt quaternion

  if (i == 0):
  #qx
    J = np.array([[ 0,   2*qy,  2*qz],
                [2*qy,  -4*qx, -2*qw],
                [2*qz,   2*qw, -4*qx]])
  if (i == 1):
  #qy
    J = np.array([[ -4*qy,  2*qx,  2*qw],
                   [ 2*qx,     0,  2*qz],
                   [-2*qw,  2*qz, -4*qy]])
  if (i == 2):
  #qz
    J = np.array([[-4*qz, -2*qw, 2*qx ],
                  [ 2*qw, -4*qz, 2*qy ],
                  [ 2*qx,  2*qy,    0 ]])
  if (i == 3):
  #qw
    J = np.array([[  0, -2*qz,  2*qy],
                [ 2*qz,     0, -2*qx],
                [-2*qy,  2*qx,     0]])

  return J

def buildFmat(dT, state,accMeas,gyroMeas):


  x,y,z,vx,vy,vz,qx,qy,qz,qw,abx,aby,abz,bx,by,bz = state.T[0]
  att = AQ.Quaternion([qx,qy,qz,qw])

  Fx = np.hstack( (dfxdx(), dfxdv(dT), 0.*dfxdq(dT,att,accMeas), dfxdaB(dT,att),np.zeros((3,3)) ) )
  #Fx = np.hstack( (dfxdx(), dfxdv(dT), np.zeros((3,10) ) ))
  Fv = np.hstack( (np.zeros((3,3)) , dfvdv(), 0*dfvdq(dT,att,accMeas), dfvdaB(dT,att),np.zeros((3,3)) ) )
  #Fv = np.hstack( (np.zeros((3,3)) , dfvdv(), np.zeros((3,10)) ) )
  Fq = np.hstack( (np.zeros((4,6)), dfqdq(dT,gyroMeas), np.zeros((4,3)), dfqdwB(dT,att) ) )
  Fb = np.zeros((6,16))
  return np.vstack((Fx,Fv,Fq,Fb))

def buildGmat(dT,att):

  Gx = np.hstack( (dfxda(dT,att), np.zeros((3,3)) ) )
  Gv = np.hstack( (dfvda(dT,att), np.zeros((3,3)) ) )
  Gq = np.hstack( (np.zeros((4,3)), dfqdw(dT,att) ) )
  Gbias = np.zeros((6,6))

  return np.vstack((Gx,Gv,Gq,Gbias))







class KF:
    def __init__(self,initialState = np.array([[0,0,0,0,0,0]],dtype=np.float64).T,initialCovariance = 1*np.eye(6,dtype=np.float64)):

        self.state = initialState
        self.cov = initialCovariance
        self.cov[3:6,3:6] = .001*np.eye(3)

    def runFilter(self,accMeas,otherMeas,dT):
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
        self._predictStep(accMeas,dT)
        # update with other measurements
        #if (len(otherMeas) > 0):
        self._updateStep(otherMeas)
        # all done
        return [self.state, self.cov]


    def _predictStep(self,accMeas,dT):
        # unpack
        #print self.state.T[0]
        x,y,z,vx,vy,vz = self.state.T[0]
        A1 = np.hstack((np.eye(3),dT*np.eye(3)))
        A2 = np.hstack((np.zeros((3,3)), np.eye(3)))
        A = np.vstack((A1,A2))
        B1 = 0.5*dT*dT*np.eye(3)
        B2 = dT*np.eye(3)
        B = np.vstack((B1,B2))
        stateHat = np.dot(A,self.state) + np.dot(B,accMeas)
        self.state = stateHat
        # noise
        RT = np.eye(3)
        R = np.dot(B,np.dot(RT,B.T))
        # Inject some noise into the estimator
        self.cov = np.dot(A,np.dot(self.cov,A.T)) + R

    def _updateStep(self,Measurements):
      # handle update
      for meas in Measurements:
        if (meas[0] == 'gps'):
          z = meas[1]
          H = np.hstack( (np.eye(3), np.zeros((3,3)) ) )
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, z - self.state[0:3,0:1])
          self.cov = np.dot(np.eye(6) - np.dot(K,H), self.cov)
        if (meas[0] == 'mag'):
          pass
          #z = np.dot(att.asRotMat.T,meas[1])
          #z[2,0] = 0
          #z = 1./(np.linalg.norm(z) + 1E-13)*z
          #h = meas[2]  # earth mag reading
          #h[2,0] = 0
          #h = 1/(np.linalg.norm(h) + 1e-13)*h # renormalize
          #axisSinAngle = np.cross(h.T,z.T)
          #angle = np.arcsin(np.linalg.norm(axisSinAngle))
          #axis = (1./(np.linalg.norm(axisSinAngle) + 1e-5))*axisSinAngle




#
#
# EKF
#
#

class EKF:

    '''

       state = [x y z vx vy vz qx qy qz qw accBias gyroBias]'


    '''

    # constructor

    def __init__(self,initialState = np.array([[0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0]],dtype=np.float64).T,initialCovariance = 1*np.eye(16,dtype=np.float64)):

        self.state = initialState
        self.cov = initialCovariance
        self.cov[3:6,3:6] = .001*np.eye(3)

    def runFilter(self,accMeas,gyroMeas,otherMeas,dT):
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
        self._predictStep(accMeas,gyroMeas,dT)
        # update with other measurements
        #if (len(otherMeas) > 0):
        self._updateStep(otherMeas)
        # all done
        return [self.state, self.cov]


    def _predictStep(self,accMeas,gyroMeas,dT):
        # unpack
        #print self.state.T[0]
        x,y,z,vx,vy,vz,qx,qy,qz,qw,abx,aby,abz,bx,by,bz = self.state.T[0]
        # repack
        att = AQ.Quaternion([qx,qy,qz,qw])
        # Rotate accel into world
        accBias = np.array([[abx,aby,abz]]).T
        acc_world = np.dot(att.asRotMat.T,accMeas-accBias)
        #print "acc_w: ",acc_world.T, qx, qy, qz, qw, abx,aby,abz
        # subtract gravity
        acc_net = acc_world - np.array([[0,0,-9.81]]).T
        # pos update
        xHat = np.array([[x,y,z]]).T + dT*np.array([[vx,vy,vz]]).T + 0.5*dT*dT*acc_net
        # vel update
        vHat = np.array([[vx,vy,vz]]).T + dT*acc_net
        # quaternion state update
        bias = np.array([[bx,by,bz]]).T
        newAtt = quatFromRotVec((gyroMeas-bias)*dT)*att
        newAtt.normalize()
        # accel bias update
        # constant
        # gyro bias update
        # constant
        self.state = np.vstack((xHat,vHat,np.array([newAtt.q]).T,accBias,bias))
        #
        # Now do covariance
        # Build "A matrix"
        #
        # TODO: Check transposes!!!
        #
        # Build F matrix
        F = buildFmat(dT, self.state,acc_net,gyroMeas)
        G = buildGmat(dT,att)
        # noise
        RT = .01*np.eye(6)
        R = np.dot(G,np.dot(RT,G.T))
        #R = 1e-6*np.eye(16)
        # Inject some noise into the estimator
        sigma_bias = .01
        R[10:16,10:16] = sigma_bias*np.eye(6)
        self.cov = np.dot(F,np.dot(self.cov,F.T)) + R

    def _updateStep(self,Measurements):
      # handle update
      for meas in Measurements:
        if (meas[0] == 'gps'):
          z = meas[1]
          H = np.hstack( (np.eye(3), np.zeros((3,13)) ) )
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, z - self.state[0:3,0:1])
          self.cov = np.dot(np.eye(16) - np.dot(K,H), self.cov)
          #print np.diag(self.cov)
        if (meas[0] == 'baro'):
          z = meas[1]
          H = np.array([[0, 0, 1, 0,0,0,0,0,0,0,0,0,0,0,0]])
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, z - np.dot(H,self.state))
          self.cov = np.dot(np.eye(16) - np.dot(K,H), self.cov)


#
#
# Multiplicative EKF
#
#
class MEKF:

    '''

       state = [x y z vx vy vz gibbsx gibbsy gibbsz accBias gyroBias]'
       inputs: gyro, accel
       measurements: GPS, baro, mag

    '''

    # constructor

    def __init__(self,initialState = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]],dtype=np.float32).T,initialCovariance = 100000*np.eye(15,dtype=np.float32),processNoise = np.diag([.1,.1,.1, 1e-5, 1e-5, 1e-5, 1.,1.,1.,1e-5, 1e-5, 1e-5])):

        self.state = initialState
        self.cov = initialCovariance
        self.cov[3:6,3:6] = .001*np.eye(3)
        self.q = AQ.Quaternion([0,0,0])
        self.processNoise = processNoise
        self.initialized = False
        self.initializationCounter = 100
        self.firstGPS = False

    def warmStart(self,acc,gyro,OtherMeas,dT):
        # accumulate measurements without touching covariance
        if (self.initializationCounter > 0):
          self.initializationCounter -= 1
          #OtherMeas.append(['acc',acc,.1*self.processNoise[6:9,6:9]])
          # Assume you're not moving fast at initialization
          OtherMeas.append(['vel',np.zeros((3,1)),.1*np.eye(3)])
          #self._predictStep(acc,gyro,dT)
          self._updateStep(OtherMeas)
          # Don't estimate biases yet
          #self.state[9:15] = np.zeros((6,1))
          self._resetQuat()
        else:
          self.initialized = True

    def runFilter(self,accMeas,gyroMeas,otherMeas,dT):
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
        if (self.initialized):
          self._predictStep(accMeas,gyroMeas,dT)
          # update with other measurements
          #if (len(otherMeas) > 0):
          otherMeas.append(['acc',accMeas,10*self.processNoise[6:9,6:9]])
          self._updateStep(otherMeas)
          # push gibbs vector onto quat
          self._resetQuat()
          # all done
        else:
          self.warmStart(accMeas,gyroMeas,otherMeas,dT)
          print "INITIALIZING MEKF!"
        return [self.state, self.cov]


    def _predictStep(self,accMeas,gyroMeas,dT):
        #
        self.propagateState(accMeas,gyroMeas,dT)
        self.propagateCovariance(dT,accMeas,gyroMeas)

    def propagateState(self,accMeas,gyroMeas,dT):
        # unpack
        x,y,z,vx,vy,vz,gbx,gby,gbz,abx,aby,abz,bx,by,bz = self.state.T[0]
        # repack
        att = self.q
        # Rotate accel into world
        accBias = np.array([[abx,aby,abz]]).T
        acc_world = np.dot(att.asRotMat.T,accMeas-accBias)
        #print "accBias_w: ",accBias.T, "gyroBias: ",np.array([[bx,by,bz]])
        # subtract gravity
        acc_net = acc_world - np.array([[0,0,-9.81]]).T
        ##
        #  Perform state updates
        ##
        # pos update
        xHat = np.array([[x,y,z]]).T + dT*np.array([[vx,vy,vz]]).T + .5*dT*dT*acc_net

        # vel update
        vHat = np.array([[vx,vy,vz]]).T + dT*acc_net
        # quaternion state update
        bias = np.array([[bx,by,bz]]).T
        gibbsHat = (gyroMeas-bias)*dT
        # Pack it up
        self.state = np.vstack((xHat,vHat,gibbsHat,accBias,bias))


    def propagateCovariance(self,dT,accMeas,gyroMeas):
        bias = self.state[12:15]
        Fcont = self._buildFmat(dT,accMeas,gyroMeas,bias)
        Gcont = self._buildGmat(dT)
        Fdisc = np.eye(15) + dT*Fcont
        # sensor noise: [ gyro gyroBias accel accelBias ]
        if (self.initialized):
          sensorNoise = self.processNoise
        else:
          sensorNoise = 1*self.processNoise
        GQGT = np.dot(Gcont, np.dot(sensorNoise,Gcont.T))
        Qdisc = dT*dT*GQGT
        self.cov = np.dot(Fdisc,np.dot(self.cov,Fdisc.T)) + Qdisc

    # build linearized state transition matrix for covariance propagation
    def _buildFmat(self,dT,acc,gyroMeas,bias):
        # build continuous time model
        dfGibbs_dGibbs = np.zeros((3,3)) #-crossMat(gyroMeas)
        dfGibbs_dGibbs = -crossMat(gyroMeas)
        #dfGibbs_dBias = -np.eye(3)
        att = quatFromRotVec(self.state[6:9])*self.q
        dfVel_dGibbs = np.dot(-att.asRotMat.T,crossMat(acc))
        dfVel_daBias = -att.asRotMat.T
        Row1 = np.hstack( (np.zeros((3,3)), np.eye(3), np.zeros((3,9)) ))
        Row2 = np.hstack( (np.zeros((3,6)), dfVel_dGibbs, dfVel_daBias, np.zeros((3,3))) )
        Row3 = np.hstack( (np.zeros((3,6)), dfGibbs_dGibbs, np.zeros((3,3)),-np.eye(3) ) )
        Fcont = np.vstack((Row1,Row2,Row3,np.zeros((6,15)) ))
        return Fcont

    def _buildGmat(self,dT):
        # noise assumed to have form v = [gyro noise, gyro bias driver, accel noise, accel bias driver]
        att = quatFromRotVec(self.state[6:9])*self.q
        dGibbs_dGyroNoise = np.eye(3)
        dGyroBias_dGyroNoiseDriver = np.eye(3)
        dVel_dAccNoise = att.asRotMat.T
        dAccBias_dAccNoiseDriver = np.eye(3)
        Row2 = np.hstack( (np.zeros((3,6)), dVel_dAccNoise, np.zeros((3,3)) ) )
        Row3 = np.hstack( ( np.eye(3), np.zeros((3,9)) ))
        Row4 = np.hstack( (np.zeros((3,9)), np.eye(3)) )
        Row5 = np.hstack( (np.zeros((3,3)), np.eye(3), np.zeros((3,6)) ) )
        Gcont = np.vstack( (np.zeros((3,12)), Row2, Row3, Row4,Row5 ) )
        return Gcont

    def _resetQuat(self):
        dq = quatFromRotVec(self.state[6:9])
        self.q = dq*self.q
        self.state[6:9] = np.zeros((3,1))

    def updateCovariance(self,K,H):
      if (self.initialized):
        self.cov = np.dot(np.eye(15) - np.dot(K,H), self.cov)

    def _updateStep(self,Measurements):
      # handle update
      attEst = quatFromRotVec(self.state[6:9])*self.q
      for meas in Measurements:
        if (meas[0] == 'gps'):
          if (self.firstGPS == False):
            self.firstGPS = True
            self.state[0] = meas[1][0]
            self.state[1] = meas[1][1]
            self.state[2] = meas[1][2]
            self.covariance[0,0] = meas[2][0,0]
            self.covariance[1,1] = meas[2][1,1]
            self.covariance[2,2] = meas[2][2,2]
            self.state[9] = 0.0
            self.state[10] = 0.0
            self.state[11] = 0.0
          else:
            z = meas[1]
            z_exp = self.state[0:3,0:1] + np.dot(attEst.asRotMat.T,meas[3])
            H = np.hstack( (np.eye(3),np.zeros((3,3)), -np.dot(attEst.asRotMat.T,crossMat(meas[3])), np.zeros((3,6)) ) )
            shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
            K = np.dot( np.dot(self.cov,H.T) , shur )
            self.state = self.state + np.dot(K, z - z_exp)
            self.updateCovariance(K,H)
            #print np.diag(self.cov)
        if (meas[0] == 'baro'):
          z = meas[1]
          H = np.array([[0, 0, 1, 0,0,0,0,0,0,0,0,0,0,0,0]])
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, z - np.dot(H,self.state))
          self.updateCovariance(K,H)
        if (meas[0] == 'mag'):
          z = meas[1]
          #z_world = np.dot(attEst.asRotMat.T,z)
          #z_world[2,0] = 0
          #z_world = (1./np.linalg.norm(z))*z_world
          #z = np.dot(attEst.asRotMat,z_world)
          magWorld = meas[3]
          #magWorld[2,0] = 0
          magWorld = (1./np.linalg.norm(magWorld))*magWorld
          expectedMag = np.dot(attEst.asRotMat,magWorld)
          err = z - expectedMag
          #print "mag err: ",err.T
          # TODO: check this math
          H = np.hstack(( np.zeros((3,6)), crossMat(z), np.zeros((3,6)) ) )
          Qmeas = meas[2]
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + Qmeas)
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, err)
          self.updateCovariance(K,H)
        if (meas[0] == 'acc'):
          accBias = np.array([self.state[9:12]]).T
          if (abs(np.linalg.norm(meas[1] - accBias) - 9.81) < 1):
            z = meas[1] - self.state[9:12]
            z = (1./(np.linalg.norm(z)+1E-13))*z
            gravWorldUnit = np.array([[0,0,-1]]).T
            gravBodyUnit = np.dot(attEst.asRotMat,gravWorldUnit)
            err = z - gravBodyUnit
            #print 'tilt err: ',err.T
            H = np.hstack( (np.zeros((3,6)), crossMat(z), np.zeros((3,6)) ) )
            Qmeas = meas[2]
            shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + Qmeas)
            K = np.dot( np.dot(self.cov,H.T) , shur )
            self.state = self.state + np.dot(K, err)
            self.updateCovariance(K,H)
        if (meas[0] == 'vel'):
          z = meas[1]
          H = np.hstack( (np.zeros((3,3)), np.eye(3), np.zeros((3,9)) ) )
          shur = np.linalg.inv( np.dot(H, np.dot(self.cov,H.T)) + meas[2] )
          K = np.dot( np.dot(self.cov,H.T) , shur )
          self.state = self.state + np.dot(K, z - self.state[3:6,0:1])
          self.updateCovariance(K,H)







def testHarness():
    testEKF = True
    if (testEKF):
      filt = EKF()
    else:
      filt = KF()
    accel_world = np.array([[0.1,.0,0]]).T
    gravMeas =  np.array([[.0,.0,-9.81]]).T
    gyro = np.array([[.0,.0,.1]]).T
    omega = np.linalg.norm(gyro)
    velocity = np.zeros((3,1))
    pos = velocity
    dT = 0.01
    att = AQ.Quaternion([0,0,0])

    for ii in range(0,1000):
      #att = AQ.quatFromAxisAngle(gyro,np.linalg.norm(gyro)*dT)*att
      accelerometer = np.dot(att.asRotMat,accel_world+gravMeas)

      att = AQ.quatFromAxisAngle(gyro,np.linalg.norm(gyro)*dT)*att
      if (ii%21 == 0):

        gps = ['gps',pos,.01*np.eye(3)]
        other = [gps]
      else:
        other = [['g',omega,np.eye(3)]]

      if (testEKF):
        stateAndCov = filt.runFilter(accelerometer+1.*np.random.randn(3,1),gyro,other,dT)
      else:
        stateAndCov = filt.runFilter(accel_world+.01*np.random.randn(3,1),other,dT)

      if 'states' in locals():
        states = np.hstack((states,stateAndCov[0]))
        covs = np.vstack((covs,np.diag(stateAndCov[1])))
      else:
        states = stateAndCov[0]
        covs = np.diag(stateAndCov[1])

      pos = pos + velocity*dT + .5*dT*dT*accel_world
      velocity = velocity + (accel_world)*dT

    handles = plt.plot(states.T[:,0:3],marker='s')
    plt.legend(['x','y','z'])
    plt.figure()
    handles = plt.plot(states.T[:,3:6])
    plt.legend(['vx','vy','vz'])
    plt.figure()
    if (testEKF):
      handles = plt.plot(states.T[:,6:10])
      plt.legend(['qx','qy','qz','qw'])
      plt.figure()
      plt.plot(covs[:,0:6])
      plt.legend(['x','y','z','vx','vy','vz','qx','qy','qz','qw'])
    else:
      plt.plot(covs[:,0:6])
      plt.legend(['x','y','z','vx','vy','vz'])

    plt.show()

def quatFromRotVec(vec):
    angle = np.linalg.norm(vec)
    if (angle == 0.):
      return AQ.Quaternion([0.,0.,0.,1.])
    axis = vec/angle
    return AQ.quatFromAxisAngle(axis,angle)


if __name__ == "__main__":
    testHarness()
