#! /usr/local/bin/python
# openGL imports
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
# common imports
import numpy as np
import time as clock
import math
# simulation imports
import Multirotor
import AeroQuaternion as AQ
import QuadrotorController
import KinematicEKF
import KalmanFilter as KF


############################
# 
#    Begin setup
#
############################
name = 'Quadrotor Visualizer'
yaw = 0
height = 0;
position = np.zeros((3,1))
attitude = [0,0,30]
attEst = [0,0,00]
cameraMode = 'CHASE_CAM'
refType = 'xyah'
yawCmd = 0.
zCmd = 10.
cutMotors = True
gpsGood = False
###################################
# Create Quadrotor object
# and initialize sim stuff
###################################
MotorHeightOffset = 0.
motorPos = [np.array([[.19,-.19,MotorHeightOffset]]).T,
            np.array([[.19,.19,MotorHeightOffset]]).T,
            np.array([[-.19,.19,MotorHeightOffset]]).T,
            np.array([[-.19,-.19,MotorHeightOffset]]).T]
Quad = Multirotor.Multirotor(fuselageMass = .5,motorPositions = motorPos) # default is quadrotor
initialAtt = AQ.Quaternion(attitude)
Quad.stateVector[0,6] = initialAtt.q[0]
Quad.stateVector[0,7] = initialAtt.q[1]
Quad.stateVector[0,8] = initialAtt.q[2]
Quad.stateVector[0,9] = initialAtt.q[3]
idx = 0
dt = 0.002
dtControl = .008
T = 1.3
numsteps = 3
maxInd = int(math.ceil(T/dt))
stateHist = np.zeros((numsteps*maxInd,13))
commands = [0.5,0.5,0.5,0.5]
# add wind
#windvel = np.array([[30.,30.,0]]).T 
windvel = np.zeros((3,1))
# run for a whil
controller = QuadrotorController.Controller()

#--------------------------------------
# Kalman initialization
#--------------------------------------
gyroNoise = .01
gyroBiasNoise = 1e-10
accelNoise = .1
accelBiasNoise = 1e-10
sensorNoise = np.diag([gyroNoise,gyroNoise,gyroNoise,
                      gyroBiasNoise, gyroBiasNoise, gyroBiasNoise,
                      accelNoise,accelNoise,accelNoise,
                      accelBiasNoise, accelBiasNoise, accelBiasNoise])
initialPos = np.array([[0,10,0]]).T
initialVel = np.array([[0,0,0]]).T
initialAtt = AQ.Quaternion(attEst)

initial_state = np.vstack((initialPos,initialVel,np.zeros((9,1))))

EKF = KF.MEKF(initialState = initial_state, processNoise = sensorNoise)
EKF.q = initialAtt

AttEstimator = KinematicEKF.AttitudeComplementaryFilter(initialAttitude=AQ.Quaternion(np.array(attEst)), timeConstant = 10.,Kbias = .00)
reference = [0.,0.,5.,0.]
time = 0.
lastTime = 0.
lastGPS = 0.
lastMAG = 0.
lastControlTime = 0.
MAG_FREQ = 80.
startTime = clock.time()
period = dt
Quad.stateVector[0,2] = 0. # initial height
Quad.stateVector[0,6:10] = AQ.Quaternion(np.array(attitude)).q
#np.random.seed([])
accMeas = np.zeros((3,1))


  

def drawQuad(att = [0,0,0], pos = np.zeros((3,1)), color = [1.,0.,0.,1.]):
    global Quad
    '''glPushMatrix() 
    xLen = 1
    yLen = 1
    zLen = 1
    glTranslate(pos[0,0],pos[1,0],pos[2,0])
    glPushMatrix()
    glRotate(att[2],0,0,1)
    glRotate(att[1],0,1,0)
    glRotate(att[0],1,0,0)'''
    # rotor blades
    drawQuad.Theta1 += Quad.motorList[0].motorState[1]*1
    drawQuad.Theta2 += Quad.motorList[1].motorState[1]*1
    drawQuad.Theta3 += Quad.motorList[2].motorState[1]*1
    drawQuad.Theta4 += Quad.motorList[3].motorState[1]*1
    emissionColor = [.5,0.,0.,.1]
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)

    # draw 4 rings
    glPushMatrix()
    glTranslatef(-1.2,1.2,0.)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidTorus(.1,1.,20,20)
    # draw prop
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(drawQuad.Theta3,0,0,1)
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(1.0,0.1,0.0)
    glVertex3f(1.0,-.1,0.0)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(-1.0,-0.1,0.0)
    glVertex3f(-1.0,.1,0.0)
    glEnd()
    glPopMatrix()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(1.2,1.2,0.)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidTorus(.1,1.,20,20)
    # draw prop
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(drawQuad.Theta2,0,0,-1)
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(1.0,0.1,0.0)
    glVertex3f(1.0,-.1,0.0)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(-1.0,-0.1,0.0)
    glVertex3f(-1.0,.1,0.0)
    glEnd()
    glPopMatrix()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(1.2,-1.2,0.)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidTorus(.1,1.,20,20)
    # draw prop
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(drawQuad.Theta1,0,0,1)
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(1.0,0.1,0.0)
    glVertex3f(1.0,-.1,0.0)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(-1.0,-0.1,0.0)
    glVertex3f(-1.0,.1,0.0)
    glEnd()
    glPopMatrix()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(-1.2,-1.2,0.)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidTorus(.1,1.,20,20)
    # draw prop
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(drawQuad.Theta4,0,0,-1)
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(1.0,0.1,0.0)
    glVertex3f(1.0,-.1,0.0)
    glVertex3f(0.0,0.0,0.0)
    glVertex3f(-1.0,-0.1,0.0)
    glVertex3f(-1.0,.1,0.0)
    glEnd()
    glPopMatrix()
    glPopMatrix()
   
    # drawBody
    glPushMatrix()
    # Rotate before Translate (otherwise weird things happen)
    glRotate(45., 0.0, 0.0, 1.0)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidCube(.5)
    glPopMatrix()
    '''
    # direction 
    glPushMatrix()
    glRotate(90,0,1,0)
    # draw base
    coneRadius = .2
    arrowLength = 2
    glDisable(GL_CULL_FACE)
    # x-axis
    q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(.2,.7,20,20)
    glPopMatrix()'''
drawQuad.Theta1 = 0
drawQuad.Theta2 = 0
drawQuad.Theta3 = 0
drawQuad.Theta4 = 0



def drawAxes(coneRadius = .2, arrowLength = 2):
    glDisable(GL_CULL_FACE)
    # x-direction 
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [1,0,0,1]);
    glRotate(90,0,1,0)
    # draw base
    coneRadius = .2
    arrowLength = 2
    glDisable(GL_CULL_FACE)
    q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(.2,.7,20,20)
    glPopMatrix()

    # y-direction 
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(90,-1,0,0)
    # draw base
    coneRadius = .2
    arrowLength = 2
    glDisable(GL_CULL_FACE)
    #q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(.2,.7,20,20)
    glPopMatrix()

    # z-direction 
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [1,1,0,1]);
    # draw base
    coneRadius = .2
    arrowLength = 2
    glDisable(GL_CULL_FACE)
    #q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(.2,.7,20,20)
    glPopMatrix()





def drawAnchor(position):
    glBegin(GL_LINES)
    glVertex3f(position[0,0],position[1,0],position[2,0])
    glVertex3f(position[0,0],position[1,0],0)
    glEnd()


def drawEnvironment():
    
    # ground grid
    glBegin(GL_LINES)
    gridSize = 1000
    step = 15
    for ii in range(-gridSize,gridSize+step,step):
       glVertex3i(gridSize,ii,0)
       glVertex3i(-gridSize,ii,0)
       glVertex3i(ii,gridSize,0)
       glVertex3i(ii,-gridSize,0)
    
    glEnd()



def runDynamics():
    global yaw
    global height 
    global windvel
    global period
    global commands
    global Quad
    global EKF
    global reference
    global position
    global attitude
    global attEst
    global startTime
    global refType
    global cutMotors
    global yawCmd
    global lastTime
    global lastGPS
    global gpsGood
    global lastMAG
    global MAG_FREQ
    global dtControl
    global lastControlTime
    global gyroNoise
    global accelNoise
    
    # timing stuff
    Time = clock.time()
    dT = Time - runDynamics.lastTime
    wind = windvel # + 10*np.random.randn(3,1)
    if (dT < period):
        return

    # else update state
    disturbance = 10
    if (Time - startTime > 2 and Time - startTime < 10):
        pass #wind = np.array([[10,0,0]]).T

    state,acc = Quad.updateState(dt,commands,windVelocity = wind,disturbance = disturbance)

    if ( Time - lastTime > 1. ):
      print dT, 'x y ht: ', state[0,0], ' ', state[0,1], ' ', state[0,2]
      lastTime = Time
    # simulate measurements
    accMeas = acc +  accelNoise*np.array([np.random.randn(3)]).T + np.array([[1.0],[.0],[.0]]) 
    gyroMeas = state.T[10:] + gyroNoise*np.array([np.random.randn(3)]).T + np.array([[.3],[.1],[.0]]) #+ np.array
    # set it askew
    accMeas = np.dot(AQ.Quaternion([0,0,0]).asRotMat,accMeas)
    gyroMeas = np.dot(AQ.Quaternion([0,0,0]).asRotMat,gyroMeas)

    attTrue = AQ.Quaternion(state[0,6:10])
    #earthMagReading = np.array([[.48407,.12519,.8660254]]).T
    earthMagReading = np.array([[.48407,.12519,.0]]).T
    #earthMagReading = np.array([[2., 0., 10. ]]).T
    earthMagReading = 1./np.linalg.norm(earthMagReading)*earthMagReading
    magMeas = np.dot(attTrue.asRotMat,earthMagReading) + .01*np.array([np.random.randn(3)]).T
    magCov = .1*np.eye(3)
    otherMeas = []
        # gps update?
    if (Time - lastControlTime > dtControl and Time - lastMAG >= 1./MAG_FREQ):
      otherMeas.append(['mag',magMeas,magCov,earthMagReading])
      lastMAG = Time
    if (Time - lastControlTime > dtControl and Time - lastGPS >= .5):
      velWorld = np.dot(attTrue.asRotMat.T,state[0:1,3:6].T)
      gpsMeas = np.vstack((state[0:1,0:3].T,velWorld)) + .01*np.array([np.random.randn(6)]).T + np.array([[.0],[.0],[.0],[.0],[.0],[-.1]])
      #print gpsMeas
      if (gpsGood):
        otherMeas.append(['gps',gpsMeas,np.diag([1,1,50,.1,.1,10])])
      lastGPS = Time
    # run attitude filter
    otherMeas.append(['baro',state[0:1,2:3],np.array([[.1]]) ])
    if (Time - lastControlTime > dtControl):
      #attitudeAndGyroBias = AttEstimator.runFilter(accMeas,gyroMeas,otherMeas,dt)
      stateAndCovariance = EKF.runFilter(accMeas,gyroMeas,otherMeas,dtControl)
      lastControlTime = Time
    # update control
    controlState = state.copy()
    qx,qy,qz,qw = EKF.q.q
    controlState[0,6] = qx
    controlState[0,7] = qy
    controlState[0,8] = qz
    controlState[0,9] = qw
    controlState[0,0] = EKF.state[0,0]
    controlState[0,1] = EKF.state[1,0]
    controlState[0,2] = EKF.state[2,0]
    #if (Time - startTime > 5 and Time - startTime <= 12.):
    #    reference = [5,2,4,0]
    '''if (Time - startTime > 12):
        reference = [0,0,15.,np.pi/2]
    if (Time - startTime > 20 and Time - startTime < 25):
        commands = controller.updateControl(dt,state,reference,'cut')
    else:'''
    if (refType == 'xyah'):
        reference[3] = yawCmd
    if (refType == 'rpya'):
        #dyrdt = state[0,4]
        #yawCmd += dyrdt
        reference[2] = yawCmd
    if (cutMotors):
       commands = controller.updateControl(dt,controlState,reference,'cut')
       #commands = controller.updateControl(dt,state,reference,'cut')
    else:
       commands = controller.updateControl(dt,controlState,reference,refType)
       #commands = controller.updateControl(dt,state,reference,refType)
    if (False):
       print 'commands: ',commands
       print 'state: ',state
       print 'dT: ',dT
    #print Time - startTimes, 'altitude:', state[0,2]
    attitude = attTrue.asEuler
    attEst = EKF.q.asEuler
    position[0,0] = state[0,0]
    position[1,0] = state[0,1]
    position[2,0] = state[0,2]
    yaw = yaw + 1
    height = 5*np.abs(np.sin(np.pi/180.*yaw))
    # only update screen at reasonable rate
    if (Time - runDynamics.lastFrameTime >= 1./20.):
       # redisplay
       glutPostRedisplay()
       runDynamics.lastFrameTime = Time
    runDynamics.lastTime = Time
runDynamics.lastTime = -.005
runDynamics.lastFrameTime = -.005

def main():
    # set up opengl stuff
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(700,700)
    glutCreateWindow(name)
    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = [10.,4.,10.,1.]
    lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    glEnable(GL_LIGHT0)
    glutDisplayFunc(display)
    glutIdleFunc(runDynamics)
    glutKeyboardFunc(keyboardHandler)
    glutMotionFunc(dragFunc)
    glutMouseFunc(mouseFunc)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(40.,1.,1.,1000.)
    glMatrixMode(GL_MODELVIEW)
    '''gluLookAt(position[0,0]-30,position[1,0]-0,position[2,0] -20,
              position[0,0],position[1,0],position[2,0],	
              3,0,2)
    glPushMatrix()'''

    # set up quad stuff
 


    glutMainLoop()
    return

def display():
    startTime = clock.time()
    #print startTime,
    global position
    global attitude
    global attEst
    global cameraMode
    global EKF
    positionEst = EKF.state[0:3,0:1]
    glLoadIdentity()
    gl_R_ned = AQ.Quaternion(np.array([0,180,-90]))
    veh_R_ned = AQ.Quaternion(attitude)
    veh_R_nedEst = AQ.Quaternion(attEst)
    veh_R_yaw = AQ.Quaternion(np.array([0,0,attitude[2]]))
    chaseCamPos = np.dot(gl_R_ned.asRotMat,np.dot(veh_R_yaw.asRotMat.T,np.array([[-30],[0],[-10]])))
    posGL = np.dot(gl_R_ned.asRotMat,position)
    posGLest = np.dot(gl_R_ned.asRotMat,positionEst)
    if (cameraMode == 'CHASE_CAM'):
       gluLookAt(posGL[0]+chaseCamPos[0],posGL[1]+chaseCamPos[1],posGL[2]+chaseCamPos[2],
                 posGL[0],posGL[1],posGL[2],	
              0,0,1)
    elif (cameraMode == 'CHASE_CAM_EST'):
       gluLookAt(posGLest[0]+chaseCamPos[0],posGLest[1]+chaseCamPos[1],posGLest[2]+chaseCamPos[2],
                 posGLest[0],posGLest[1],posGLest[2],	
              0,0,1)
    elif (cameraMode == 'GROUND_CAM'):
       gluLookAt(0,-30,2,
                 posGL[0],posGL[1],posGL[2],	
                 0,0,1)
    elif (cameraMode == 'ONBOARD_CAM'):
       veh_rider = np.dot(gl_R_ned.asRotMat,position + np.dot(veh_R_ned.asRotMat.T,np.array([[2],[0],[-.20]])))
       veh_forward = posGL + np.dot(gl_R_ned.asRotMat,np.dot(veh_R_ned.asRotMat.T,np.array([[1000],[0],[0]])))
       veh_up = np.dot(gl_R_ned.asRotMat,   np.dot(veh_R_ned.asRotMat.T,np.array([[0],[0],[-1]])))
       gluLookAt(veh_rider[0],veh_rider[1],veh_rider[2],	
                 veh_forward[0],veh_forward[1],veh_forward[2],	
                 veh_up[0],veh_up[1],veh_up[2])
    else:
       gluLookAt(0,-30,10,
                 0,0,0,	
              0,3,1)
    glPushMatrix()

    glPushMatrix()
    # rotate into aero convention
    glRotate(180.,1.,1.,0)


    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glColor4f(0.,0.,1.,.8)
    color = [0,0.,1.,1.]
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color)
    # draw ground
    drawEnvironment()
    color = [1.,0.,0.,1.]
    emissionColor = [.5,0.,0.,.1]
    drawAxes()
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    #glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    # push quadrotor position on stack
    glPushMatrix() 
    glTranslate(position[0,0],position[1,0],position[2,0])
    glRotate(attitude[2],0,0,1)
    glRotate(attitude[1],0,1,0)
    glRotate(attitude[0],1,0,0)
    # draw quadrotor
    drawQuad(color = [1.,0.,0.,1.])
    #Shark.drawShark()
    # draw body axes
    drawAxes()
    glPopMatrix()

    # push quadrotor estimated  position on stack
    glPushMatrix() 
    color = [1.,1.,0.,1.]
    glMaterialfv(GL_FRONT,GL_EMISSION,color)

    #glTranslate(position[0,0],position[1,0],position[2,0])
    glTranslate(positionEst[0,0],positionEst[1,0],positionEst[2,0])
    attEKF = EKF.q
    glRotate(attEKF.asEuler[2],0,0,1)
    glRotate(attEKF.asEuler[1],0,1,0)
    glRotate(attEKF.asEuler[0],1,0,0)
    #glRotate(attEst[2],0,0,1)
    #glRotate(attEst[1],0,1,0)
    #glRotate(attEst[0],1,0,0)
    color = [1.,0.,0.,1.]
    glMaterialfv(GL_FRONT,GL_EMISSION,color)

    # draw quadrotor
    drawQuad(color = [0.,1.,0.,1.])
    #Shark.drawShark()
    # draw body axes
    drawAxes()
    glPopMatrix()


    # change color
    color = [0.0,1.,0.,1.]
    glMaterialfv(GL_FRONT,GL_EMISSION,color)
    # draw anchor
    drawAnchor(position)
    glPopMatrix()
    glPopMatrix()
    glutSwapBuffers()
    #print clock.time() - startTime

    return


def keyboardHandler(key,x,y):
    global cameraMode
    global yawCmd
    global zCmd
    global reference
    global refType
    global cutMotors
    global gpsGood
    speed = 15.
    if (key == 'c'):
       cameraMode = 'CHASE_CAM'
    if (key == 'C'):
       cameraMode = 'CHASE_CAM_EST'
    if (key == 'g'):
       cameraMode = 'GROUND_CAM'
    if (key == 'f'):
       cameraMode = 'FIXED_CAM'
    if (key == 'o'):
       cameraMode = 'ONBOARD_CAM'
    if (key == '1'):
       yawCmd -= .2
    if (key == '2'):
       yawCmd += .2
    if (key == 'h'):
       cutMotors = False
       reference = [0., 0., zCmd, 0.]
       yawCmd = 0.
       refType = 'xyah'
    if (key == 'G'):
       if (gpsGood):
         gpsGood = False
       else:
         gpsGood = True
    if (key == 'L'):
       zCmd = 5.
       reference = [position[0,0], position[1,0], 0., yawCmd]
    if (key == 'K'):
       cutMotors = True
    if (key == 'k'):
       cutMotors = False
    if (key == 'u'):
       zCmd += 1.
       reference[2] = zCmd
    if (key == 'd'):
       zCmd -= 1.
       reference[2] = zCmd
    if (key == 'w'):
       reference = [position[0,0] + speed*np.cos(yawCmd) , position[1,0] + speed*np.sin(yawCmd) , zCmd, yawCmd]
    if (key == 'a'):
       reference = [position[0,0] + speed*np.sin(yawCmd) , position[1,0] - speed*np.cos(yawCmd), zCmd, yawCmd]
    if (key == 's'):
       reference = [position[0,0] - speed*np.sin(yawCmd) , position[1,0] + speed*np.cos(yawCmd), zCmd, yawCmd]
    if (key == 'z'):
       reference = [position[0,0] - speed*np.cos(yawCmd) , position[1,0] - speed*np.sin(yawCmd), zCmd, yawCmd]

def dragFunc(x,y):
   global reference
   global dragStart_x
   global dragStart_y
   global yawCmd
   global zCmd
   #print 'x: ', x, 'y: ',y
   xRef = max(min(.01*(x-dragStart_x),.5), -.5)
   yRef = max(min(.01*(y-dragStart_y),.5), -.5)
   reference = [xRef, yRef, yawCmd,zCmd]

def mouseFunc(button,state,x,y):
   global dragStart_x
   global dragStart_y
   global refType
   global reference 
   global position
   global zCmd
   if (state == GLUT_DOWN):
      dragStart_x = x
      dragStart_y = y
      refType = 'rpya'
      reference = [0,0, reference[3],zCmd]
   else:
      #gl_R_ned = AQ.Quaternion(np.array([0,180,-90]))
      #posGL = np.dot(gl_R_ned.asRotMat,position)
      refType = 'xyah'
      reference = [position[0,0],position[1,0],zCmd,yawCmd]
      #print 'reference: ', reference

if __name__ == '__main__': main()

