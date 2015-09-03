import numpy as np
import AeroQuaternion as AQ
import math


def angleFromTo(angle, min, max):
    while(angle < min):
        angle += 2*math.pi;
    while(angle >= max):
        angle -= 2*math.pi;
    return angle;



class PIDgains:
    Kp = 0.
    Ki = 0.
    Kd = 0.
    def __init__(self):
       self.Kp = 0.
       self.Ki = 0.
       self.Kd = 0.

class GainBucket:
    def __init__(self):
       self.rollRate = PIDgains()
       self.pitchRate = PIDgains()
       self.yawRate = PIDgains()
       self.roll = PIDgains()
       self.pitch = PIDgains()
       self.yaw = PIDgains()
       self.alt = PIDgains()
       self.vx = PIDgains()
       self.vy = PIDgains()
       self.vz = PIDgains()
       self.x = PIDgains()
       self.y = PIDgains()


class SecondOrderFilter:
    '''
       class for performing second-order lowpass filter operation
       supports slew rate limiting 
    '''
    def __init__(self, wn = 10, maxSlew = 10000000, minSlew = -10000000, wrap_pi = False):
        self.wn = wn
        self.v = 0
        self.x = 0
        self.maxSlew = maxSlew
        if (minSlew < 0):
            self.minSlew = minSlew
        else:
            self.minSlew = -minSlew
        self.wrap_pi = wrap_pi

    def filterAngle(self,input,dt):
        #        if (self.wrap_pi):
            #            print "input", input
            #            print "self.x", self.x
            #            abs_diff = np.abs(input-self.x)
            #            if (abs_diff>np.pi):
            #                if (input>=0):
            #                    input-=2*np.pi
            #                elif (input<0):
            #                    input+=2*np.pi
            #            print "fixd.x", self.x, '\n'
        #a =  -2*self.wn*self.v - (self.wn**2)*self.x + (self.wn**2)*input
        a =  -2*self.wn*self.v + (self.wn**2)*angleFromTo(input - self.x,-np.pi,np.pi) # <--- Important addition, makes sure lowpassed angles go the right way!
        self.x = self.x + self.v*dt
        self.v = max(min(self.v + a*dt,self.maxSlew),self.minSlew)
        return [self.x, self.v]

    def filter(self,input,dt):
        #        if (self.wrap_pi):
            #            print "input", input
            #            print "self.x", self.x
            #            abs_diff = np.abs(input-self.x)
            #            if (abs_diff>np.pi):
            #                if (input>=0):
            #                    input-=2*np.pi
            #                elif (input<0):
            #                    input+=2*np.pi
            #            print "fixd.x", self.x, '\n'
        #a =  -2*self.wn*self.v - (self.wn**2)*self.x + (self.wn**2)*input
        a =  -2*self.wn*self.v + (self.wn**2)*(input - self.x) # <--- Important addition, makes sure lowpassed angles go the right way!
        self.x = self.x + self.v*dt
        self.v = max(min(self.v + a*dt,self.maxSlew),self.minSlew)
        return [self.x, self.v]


    def reset(self):
       # resets values (but not wn) to zero
       self.x = 0
       self.v = 0


class Controller:

    gains = GainBucket()
    maxClimbRate = 1
    maxDescRate = -.25
    referenceAngles = []
    referenceRates = []
    wnCmd = 12.
    wnYawCmd = .8
    RollCmdFilter = SecondOrderFilter(wn=wnCmd)
    PitchCmdFilter = SecondOrderFilter(wn=wnCmd)
    YawCmdFilter = SecondOrderFilter(wn=wnYawCmd, wrap_pi=True)
    HtCmdFilter = SecondOrderFilter(wn=10,maxSlew = 3.,minSlew = -3)


#0.4, 0.13, 0.08, .0054);
    def __init__(self):
        # angular rate gains
        self.gains.rollRate.Kp = 0.2
        self.gains.rollRate.Ki = 0.003
        self.gains.rollRate.Kd = 0.1
        self.gains.pitchRate.Kp = 0.4
        self.gains.pitchRate.Ki = 0.03
        self.gains.pitchRate.Kd = 0.1
        self.gains.yawRate.Kp = .4
        self.gains.yawRate.Ki = 0.03
        self.gains.yawRate.Kd = 0.01
        # attitude gains
        self.gains.roll.Kp = .2
        self.gains.roll.Ki = .1
        self.gains.pitch.Kp = .2
        self.gains.pitch.Ki = .1
        self.gains.yaw.Kp = .3
        self.gains.yaw.Ki = .01
        # position
        self.gains.alt.Kp = .2
        self.gains.alt.Ki = .005
        self.gains.alt.Kd = .1
        self.gains.x.Kp = .5
        self.gains.x.Ki = .01
        self.gains.x.Kd = .0
        self.gains.y.Kp = .5
        self.gains.y.Ki = .01
        self.gains.y.Kd = .0
        # velocity
        self.gains.vx.Kp = .2
        self.gains.vx.Ki = .01
        self.gains.vx.Kd = .01
        self.gains.vy.Kp = .3
        self.gains.vy.Ki = .01
        self.gains.vy.Kd = .01
        self.gains.vz.Kp = .8
        self.gains.vz.Ki = .01
        self.gains.vz.Kd = .0
        # filtered values
        self.filteredRollCmd = 0.
        self.filteredPitchCmd = 0.
        self.filteredYawCmd = 0.
        self.filteredHtCmd = 0.
        self.filteredWxCmd = 0.
        self.filteredWyCmd = 0.
        self.filteredWzCmd = 0.
        self.filteredZdotCmd = 0.
        # integrals
        self.rollErrorInt = 0.
        self.pitchErrorInt = 0.
        self.yawErrorInt = 0.
        self.yawRateErrorInt = 0.
        self.heightErrorInt = 0.
        self.posXerrorInt = 0.
        self.posYerrorInt = 0.
        # last rates
        self.lastWxError = 0.
        self.lastWyError = 0.
        self.lastWzError = 0.
        self.lastZdotError = 0.
        self.filtpDotErr = 0.
        self.filtqDotErr = 0.
        self.filtrDotErr = 0.
        self.filtZddotErr = 0.
        self.lastvxErr = 0.
        self.lastvyErr = 0.
        self.lastvzErr = 0.
        
        

    def updateControl(self,dT,state,reference,refType):
        x,y,z,u,v,w,qx,qy,qz,qw,p,q,r = state[0]
        # extract eulers for control
        attitude = AQ.Quaternion(np.array([qx,qy,qz,qw])) 
        worldVel = np.dot(attitude.asRotMat.T,np.array([[u],[v],[w]]))
        roll,pitch,yaw = np.pi/180*attitude.asEuler # convert to rad
        if (refType == 'rpya'):
            rollRef,pitchRef,yawRef,hRef = reference
            rollRef = max(min(rollRef,.5),-.5)
            pitchRef = max(min(pitchRef,.5),-.5)
            yawRef = angleFromTo(yawRef,-np.pi,np.pi)
            self.posXerrorInt = 0.
            self.posYerrorInt = 0.
            self.heightErrorInt = 0.
            self.yawErrorInt = 0.

        elif (refType == 'xyah' or refType == 'cut'):
            # Unpack references
            xRef,yRef,zRef,yawRef = reference
            yawRef = angleFromTo(yawRef,-np.pi,np.pi)
            # compute errors
            xErr = xRef - x
            yErr = yRef - y
            hRef = zRef
            zErr = -hRef - z
            yawErr = yawRef - yaw
            #print 'errors (x, y, z, y):', xErr,yErr,zErr,yawErr
            # calculate error integrals in world frame
            self.posXerrorInt = self.posXerrorInt + xErr*dT
            self.posYerrorInt = self.posYerrorInt + yErr*dT
            self.heightErrorInt = self.heightErrorInt + zErr*dT
            self.yawErrorInt = self.yawErrorInt + yawErr*dT
            # rotate errors and integrals into body frame
            xErrBody =  xErr*np.cos(yaw) + yErr*np.sin(yaw)
            yErrBody = -xErr*np.sin(yaw) + yErr*np.cos(yaw)
            xErrIntBody =  self.posXerrorInt*np.cos(yaw) + self.posYerrorInt*np.sin(yaw)
            yErrIntBody = -self.posXerrorInt*np.sin(yaw) + self.posYerrorInt*np.cos(yaw)
            # control gains
            KiPos = 0.#self.gains.x.Ki
            KpPos = self.gains.x.Kp
            KpVel = self.gains.vx.Kp
            MAX_ANGLE = .4
            MAX_VEL = 10.
            MAX_YR = 1.
            # Velocity gains from reference positions
            vxCmd = KiPos*xErrIntBody + KpPos*xErrBody
            vyCmd = KiPos*yErrIntBody + KpPos*yErrBody
            vMag = np.sqrt(vxCmd**2 + vyCmd**2)
            saturateFlag = False
            if (vMag > MAX_VEL):
                vxCmd = MAX_VEL*vxCmd/vMag
                vyCmd = MAX_VEL*vyCmd/vMag
                saturateFlag = True
            vzCmd = min(max(self.gains.alt.Ki*self.heightErrorInt + self.gains.alt.Kp*zErr,-MAX_VEL),MAX_VEL)
            yrCmd = min(max(self.gains.yaw.Ki*self.yawErrorInt + self.gains.yaw.Kp*yawErr,-MAX_YR),MAX_YR)
            #print 'commands (vx, vy, vz, yr):', vxCmd,vyCmd,vzCmd,yrCmd
            # anti-windup
            '''if(KiPos > 0):
             if(vyCmd == MAX_VEL and yErrBody > 0.):
               yErrIntBody  = 1./KiPos*(MAX_VEL - KpPos*yErrBody)
               saturateFlag = True
             elif(vyCmd == -MAX_ANGLE and yErrBody < 0.):
               yErrIntBody  = 1./KiPos*(-MAX_VEL - KpPos*yErrBody)
               saturateFlag = True
             if(vxCmd == MAX_VEL and xErrBody > 0.):
               xErrIntBody  = 1./KiPos*(MAX_VEL - KpPos*xErrBody)
               saturateFlag = True
             elif(vxCmd == -MAX_VEL and xErrBody < 0.):
               xErrIntBody  = 1./KiPos*(-MAX_VEL - KpPos*xErrBody)
               saturateFlag = True'''
            if(saturateFlag):
               #self.posXerrorInt = np.cos(yaw)*xErrIntBody - np.sin(yaw)*yErrIntBody
               #self.posYerrorInt = np.sin(yaw)*xErrIntBody + np.cos(yaw)*yErrIntBody
               self.posXerrorInt -= xErr*dT
               self.posYerrorInt -= yErr*dT
            # handle z
            if(self.gains.alt.Ki > 0):
             if(vzCmd == MAX_VEL and zErr > 0.):
               self.heightErrorInt -= zErr*dT 
             elif(vzCmd == -MAX_VEL and zErr < 0.):
               self.heightErrorInt -= zErr*dT
            # handle yaw
            if(self.gains.yaw.Ki > 0):
             if(yrCmd == MAX_YR and yawErr > 0.):
               self.yawErrorInt -= yawErr*dT 
             elif(yrCmd == -MAX_YR and yawErr < 0.):
               self.yawErrorInt -= yawErr*dT

            # form pitch and roll commands from velocity commands
            vxErr = (vxCmd - (worldVel[0,0]*np.cos(yaw) + worldVel[1,0]*np.sin(yaw)))
            vyErr = (vyCmd - (-worldVel[0,0]*np.sin(yaw) + worldVel[1,0]*np.cos(yaw)))
            vzErr = (vzCmd - worldVel[2,0])
            #print vxErr, vyErr,vzErr
            # derivative terms
            vxErrDeriv = (vxErr - self.lastvxErr)/dT
            vyErrDeriv = (vyErr - self.lastvyErr)/dT
            vzErrDeriv = (vzErr - self.lastvzErr)/dT
            self.lastvxErr = vxErr
            self.lastvyErr = vyErr
            self.lastvzErr = vzErr
            
            rollRef = min(max(self.gains.vy.Kp*vyErr + self.gains.vy.Kd*vyErrDeriv,-MAX_ANGLE),MAX_ANGLE)
            pitchRef = min(max(-self.gains.vx.Kp*vxErr - self.gains.vx.Kd*vxErrDeriv,-MAX_ANGLE),MAX_ANGLE)
            collective = min(max( .43 - self.gains.alt.Kp*vzErr - self.gains.alt.Kd*vzErrDeriv, 0),1)
            #print 'coll:',collective
            '''# anti-windup by backsolve
            saturateFlag = False
            if(KiPos > 0):
             if(rollRef == MAX_ANGLE and yErrBody > 0.):
               yErrIntBody  = 1./KiPos*(MAX_ANGLE - KpPos*yErrBody + KpVel*v)
               saturateFlag = True
             elif(rollRef == -MAX_ANGLE and yErrBody < 0.):
               yErrIntBody  = 1./KiPos*(-MAX_ANGLE - KpPos*yErrBody + KpVel*v)
               saturateFlag = True
             if(pitchRef == MAX_ANGLE and xErrBody > 0.):
               xErrIntBody  = -1./KiPos*(MAX_ANGLE + KpPos*xErrBody - KpVel*v)
               saturateFlag = True
             elif(pitchRef == -MAX_ANGLE and xErrBody < 0.):
               xErrIntBody  = -1./KiPos*(-MAX_ANGLE + KpPos*xErrBody - KpVel*v)
               saturateFlag = True
             if(saturateFlag):
               self.posXerrorInt = np.cos(yaw)*xErrIntBody - np.sin(yaw)*yErrIntBody
               self.posYerrorInt = np.sin(yaw)*xErrIntBody + np.cos(yaw)*yErrIntBody'''
        elif (refType == 'rpYRt'):
            rollRef,pitchRef,yawRateRef,collective = reference
            #print 'reference',reference
            yawRef = yaw
            hRef = -z
            
            # end anti-windup
        #print 'attitude ', attitude.asEuler
        # Prefilter commanded angles

        # Ensure Yaw between -PI and PI
        yawRef = angleFromTo(yawRef, -math.pi, math.pi)

        # update filtered states
        self.filteredRollCmd, self.filteredWxCmd  = self.RollCmdFilter.filterAngle(rollRef,dT)
        self.filteredPitchCmd, self.filteredWyCmd = self.PitchCmdFilter.filterAngle(pitchRef,dT)
        self.filteredYawCmd, self.filteredWzCmd   = self.YawCmdFilter.filterAngle(yawRef,dT)
        self.filteredHtCmd, self.filteredZdotCmd  = self.HtCmdFilter.filter(hRef,dT)
        
        
        ################################
        ## BEGIN LOW-LEVEL CONTROLLER
        ################################
        # calculate errors
        rollError = self.filteredRollCmd - roll
        pitchError = self.filteredPitchCmd - pitch
        yawError = angleFromTo(self.filteredYawCmd - yaw, -math.pi, math.pi)
        #print yawError
        self.rollErrorInt += rollError*dT
        self.pitchErrorInt += pitchError*dT


        rollRateError = self.filteredWxCmd - p
        pitchRateError = self.filteredWyCmd - q
        if (refType == 'rpYRt'):
           yawRateError = angleFromTo(yawRateRef - r, -math.pi, math.pi)
        else:
           yawRateError =  - r
        heightError = self.filteredHtCmd -(-z)
        heightRateError = self.filteredZdotCmd - (-w) 
        #print 'err ht: ',heightError,'ref: ',reference,'filt ht cmd: ',self.filteredHtCmd,'href: ',hRef

        alf = .5
        self.filtpDotErr = .5*self.filtpDotErr + (1-alf)*(rollRateError-self.lastWxError)/dT
        self.filtqDotErr = .5*self.filtqDotErr + (1-alf)*(pitchRateError-self.lastWyError)/dT
        self.filtrDotErr = .5*self.filtrDotErr + (1-alf)*(yawRateError-self.lastWzError)/dT
        self.filtZddotErr = .5*self.filtZddotErr + (1-alf)*(heightRateError-self.lastZdotError)/dT
        # update last rate errors
        self.lastWxError = rollRateError
        self.lastWyError = pitchRateError
        self.lastWzError = yawRateError
        self.lastZdotError = heightRateError
        # done with errors
        trimThrottle = .43 # Butt number
        #print 'err, rpyh:', rollError,pitchError,yawError,heightError
        #print 'gains rpy: ',self.gains.roll.Kp,self.gains.pitch.Kp,self.gains.yaw.Kp
        # Error integrals
        ''' Motor numbering convention:
          _          _
         /1\        /2\ 
         \_/    ^   \_/
           \  Front /
            \      /
             \____/
             |    |   Top view
             |____|
            /      \
           /        \
         _/          \_
        /4\          /3\
        \_/          \_/
        '''
        if (refType == 'rpYRt' or refType == 'xyah'):  # Control mixing for roll pitch yawRate throttle commands
           
           out1 = collective \
              + (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              + (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              - (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt + self.gains.yawRate.Kd*self.filtqDotErr)     

           out2 = collective  \
              - (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              + (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              + (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt + self.gains.yawRate.Kd*self.filtqDotErr) 

           out3 = collective  \
              - (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              - (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              - (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt + self.gains.yawRate.Kd*self.filtqDotErr) 

           out4 = collective  \
              + (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              - (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              + (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt + self.gains.yawRate.Kd*self.filtqDotErr) 
        elif (refType == 'cut'):
           out1 = 0
           out2 = 0
           out3 = 0
           out4 = 0
           self.RollCmdFilter.reset()
           self.PitchCmdFilter.reset()
           self.YawCmdFilter.reset()
           self.HtCmdFilter.reset()  
        else: # Control mixing for other modes

           out1 = trimThrottle + self.gains.alt.Kp*heightError + self.gains.alt.Ki*self.heightErrorInt + self.gains.alt.Kd*heightRateError  \
              + (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              + (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              - (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt\
                  + self.gains.yawRate.Kd*self.filtqDotErr)     

           out2 = trimThrottle + self.gains.alt.Kp*heightError + self.gains.alt.Ki*self.heightErrorInt + self.gains.alt.Kd*heightRateError  \
              - (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              + (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              + (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt\
                  + self.gains.yawRate.Kd*self.filtqDotErr) 

           out3 = trimThrottle + self.gains.alt.Kp*heightError + self.gains.alt.Ki*self.heightErrorInt + self.gains.alt.Kd*heightRateError  \
              - (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              - (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              - (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt\
                  + self.gains.yawRate.Kd*self.filtqDotErr) 

           out4 = trimThrottle + self.gains.alt.Kp*heightError + self.gains.alt.Ki*self.heightErrorInt + self.gains.alt.Kd*heightRateError  \
              + (self.gains.roll.Kp*rollError + self.gains.roll.Ki*self.rollErrorInt + self.gains.rollRate.Kp*rollRateError + self.gains.rollRate.Kd*self.filtpDotErr) \
              - (self.gains.pitch.Kp*pitchError + self.gains.pitch.Ki*self.pitchErrorInt + self.gains.pitchRate.Kp*pitchRateError + self.gains.pitchRate.Kd*self.filtqDotErr)\
              + (self.gains.yaw.Kp*yawError + self.gains.yaw.Ki*self.yawErrorInt + self.gains.yawRate.Kp*yawRateError + self.gains.yawRate.Ki*self.yawRateErrorInt\
                  + self.gains.yawRate.Kd*self.filtqDotErr) 
        
        # Saturate outputs (this is dumb way to do it, but works if you're not maneuvering like crazytown. A better, but more involved way to do this is
        # by prioritizing pitch and roll authority over yaw and collective, essentially preserving the differences between the four outputs, making them all higher or lower
        # in order to keep them from saturating. I do this in my Arduino code, which I've included in the comments below, but haven't implemented in python. -Marcus
        if (out1 > 1):
           out1 = 1
        elif (out1 < 0):
           out1 = 0

        if (out2 > 1):
           out2 = 1
        elif (out2 < 0):
           out2 = 0

        if (out3 > 1):
           out3 = 1
        elif (out3 < 0):
           out3 = 0

        if (out4 > 1):
           out4 = 1
        elif (out4 < 0):
           out4 = 0

        #print ' output: ',np.array([out1,out2,out3,out4])
        #print 'filteredYawCommand: ', self.filteredYawCmd
        return np.array([out1,out2,out3,out4])




def unwrapError(angle):
    
    while(angle <= -np.pi):
        angle += 2*math.pi;
    while(angle >= np.pi):
        angle -= 2*math.pi;
    return angle;



'''
C++ code to do smarter output saturation

// Smart clip (prioritize pitch and roll over yaw and heave)

    posMargin = max(posMargin, MotorOut1 - _MAX_THRO_);
    posMargin = max(posMargin, MotorOut2 - _MAX_THRO_);
    posMargin = max(posMargin, MotorOut3 - _MAX_THRO_);
    posMargin = max(posMargin, MotorOut4 - _MAX_THRO_);
    negMargin = max(negMargin, _MIN_COLL_ - MotorOut1);
    negMargin = max(negMargin, _MIN_COLL_ - MotorOut2);
    negMargin = max(negMargin, _MIN_COLL_ - MotorOut3);
    negMargin = max(negMargin, _MIN_COLL_ - MotorOut4);

    int span = posMargin + negMargin;

    if (posMargin <= 0 && negMargin <= 0 ) { //no clipping necessary
      MotorOut1Sat = MotorOut1;
      MotorOut2Sat = MotorOut2;
      MotorOut3Sat = MotorOut3;
      MotorOut4Sat = MotorOut4;
    } else if (posMargin > 0 && span <= 0){ // simple high side clip
      MotorOut1Sat = MotorOut1 - posMargin;
      MotorOut2Sat = MotorOut2 - posMargin;
      MotorOut3Sat = MotorOut3 - posMargin;
      MotorOut4Sat = MotorOut4 - posMargin;
      // prevent windup
      pitchRate_errInt = lastPitchRateErrInt;
      rollRate_errInt = lastRollRateErrInt;
      yawRate_errInt = lastYawRateErrInt;
      pitchIntegral = lastPitchIntegral;
      rollIntegral = lastRollIntegral;
    } else if (negMargin > 0 && span <= 0){ // simple low side clip
      MotorOut1Sat = MotorOut1 + negMargin;
      MotorOut2Sat = MotorOut2 + negMargin;
      MotorOut3Sat = MotorOut3 + negMargin;
      MotorOut4Sat = MotorOut4 + negMargin;
      // prevent windup
      pitchRate_errInt = lastPitchRateErrInt;
      rollRate_errInt = lastRollRateErrInt;
      yawRate_errInt = lastYawRateErrInt;
      pitchIntegral = lastPitchIntegral;
      rollIntegral = lastRollIntegral;    
    } else { // two-sided clipping, probably will never see this. Covers the case where the difference in commanded inputs are greater than the useable range.
      MotorOut1Sat = map(MotorOut1,_MIN_COLL_ - negMargin,_MAX_THRO_ + posMargin, _MIN_COLL_,_MAX_THRO_);
      MotorOut2Sat = map(MotorOut2,_MIN_COLL_ - negMargin,_MAX_THRO_ + posMargin, _MIN_COLL_,_MAX_THRO_);
      MotorOut3Sat = map(MotorOut3,_MIN_COLL_ - negMargin,_MAX_THRO_ + posMargin, _MIN_COLL_,_MAX_THRO_);
      MotorOut4Sat = map(MotorOut4,_MIN_COLL_ - negMargin,_MAX_THRO_ + posMargin, _MIN_COLL_,_MAX_THRO_);
      // prevent windup
      pitchRate_errInt = lastPitchRateErrInt;
      rollRate_errInt = lastRollRateErrInt;
      yawRate_errInt = lastYawRateErrInt;
      pitchIntegral = lastPitchIntegral;
      rollIntegral = lastRollIntegral;
    }  
'''

