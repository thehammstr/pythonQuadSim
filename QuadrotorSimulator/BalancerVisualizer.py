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
import InvertedPendulum as IP
import AeroQuaternion as AQ
import QuadrotorController
import KalmanFilter as KF
import drawingUtils

############################
#
#    Begin setup
#
############################
if __name__ == "__main__":
    name = 'Inverted Pendulum Visualizer'
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

    robot = IP.InvertedPendulum() # default is quadrotor
    startTime = clock.time()
    dt = 0.002
    period = dt

def runDynamics():
    global period
    global commands
    global robot

    # timing stuff
    Time = clock.time()
    dT = Time - runDynamics.lastTime
    if (dT < period):
        return
    commands = [0,0]
    state,acc = robot.updateState(dt,commands)

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
    # set up quad stuff

    glutMainLoop()
    return

def display():

    startTime = clock.time()
    global cameraMode
    global robot
    position = robot.position()
    attitude = robot.attitude()
    glLoadIdentity()
    gl_R_ned = AQ.Quaternion(np.array([0,180,-90]))
    veh_R_ned = AQ.Quaternion(attitude)
    veh_R_nedEst = AQ.Quaternion(attEst)
    veh_R_yaw = AQ.Quaternion(np.array([0,0,attitude[2]]))
    chaseCamPos = np.dot(gl_R_ned.asRotMat,np.dot(veh_R_yaw.asRotMat.T,np.array([[-5],[0],[-2]])))
    posGL = np.dot(gl_R_ned.asRotMat,position)
    if (cameraMode == 'CHASE_CAM'):
       gluLookAt(posGL[0]+chaseCamPos[0],posGL[1]+chaseCamPos[1],posGL[2]+chaseCamPos[2],
                 posGL[0],posGL[1],posGL[2],
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
    # rotate into aero convention
    glRotate(180.,1.,1.,0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glColor4f(0.,0.,1.,.8)
    color = [0,0.,1.,1.]
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color)
    # draw ground
    drawingUtils.drawEnvironment()
    drawingUtils.drawAxes(.01,.1)
    # draw quadrotor
    color = [1.,1.,0.,1.]
    robot.draw()
    glPopMatrix()
    glutSwapBuffers()
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

