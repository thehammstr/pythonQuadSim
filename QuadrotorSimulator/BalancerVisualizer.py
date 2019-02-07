#! /usr/local/bin/python
# openGL imports
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
# common imports
import numpy as np
import time as clock
# simulation imports
import InvertedPendulum as IP
import AeroQuaternion as AQ
import drawingUtils

class MouseHandler:
    drag_start_x = 0
    drag_start_y = 0
    vx_ref = 0
    yaw_rate_ref = 0
    camera_mode = "CHASE_CAM"

############################
#
#    Begin setup
#
############################
def setup():
    global name
    global period
    global robot
    global dt
    global mouse_handler
    name = 'Inverted Pendulum Visualizer'
    yaw = 0
    height = 0;
    position = np.zeros((3,1))
    attitude = [0,0,30]
    attEst = [0,0,00]
    mouse_handler = MouseHandler()
    ###################################
    # Create Robot object
    # and initialize sim stuff
    ###################################

    robot = IP.InvertedPendulum()
    robot.set_attitude_rad(0, .2, 0)
    startTime = clock.time()
    dt = 0.001
    period = dt

def runDynamics():
    global period
    global mouse_handler
    global robot

    # timing stuff
    Time = clock.time()
    dT = Time - runDynamics.lastTime
    if (dT < period):
        return
    commands = [mouse_handler.vx_ref, mouse_handler.yaw_rate_ref]
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
    global mouse_handler
    global robot
    position = robot.position()
    attitude = robot.attitude_rad()
    attitude_deg = 180 / np.pi * attitude
    glLoadIdentity()
    gl_R_ned = AQ.Quaternion(np.array([0,180,-90]))
    veh_R_ned = AQ.Quaternion(attitude_deg)
    veh_R_yaw = AQ.Quaternion(np.array([0,0,attitude_deg[2]]))
    chaseCamPos = np.dot(gl_R_ned.asRotMat,np.dot(veh_R_yaw.asRotMat.T,np.array([[-5],[0],[-2]])))
    posGL = np.dot(gl_R_ned.asRotMat,position)
    if (mouse_handler.camera_mode == 'CHASE_CAM'):
       gluLookAt(posGL[0]+chaseCamPos[0],posGL[1]+chaseCamPos[1],posGL[2]+chaseCamPos[2],
                 posGL[0],posGL[1],posGL[2],
              0,0,1)
    elif (mouse_handler.camera_mode == 'GROUND_CAM'):
       gluLookAt(0,-10,2,
                 posGL[0],posGL[1],posGL[2],
                 0,0,1)
    elif (mouse_handler.camera_mode == 'ONBOARD_CAM'):
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
    # draw robot
    color = [1.,1.,0.,1.]
    robot.draw()
    glPopMatrix()
    glutSwapBuffers()
    return


def keyboardHandler(key_literal,x,y):
    key = key_literal.decode('UTF-8')
    print("Keyboard handler",key == 'g')
    global mouse_handler
    if (key == 'c'):
       mouse_handler.camera_mode = 'CHASE_CAM'
    if (key == 'C'):
       mouse_handler.camera_mode = 'CHASE_CAM_EST'
    if (key == 'g'):
       print("GGGGGGGG")
       mouse_handler.camera_mode = 'GROUND_CAM'
    if (key == 'f'):
       mouse_handler.camera_mode = 'FIXED_CAM'
    if (key == 'o'):
       mouse_handler.camera_mode = 'ONBOARD_CAM'

def dragFunc(x,y):
   global mouse_handler
   #print 'x: ', x, 'y: ',y
   mouse_handler.vx_ref = max(min(-.1*(y-mouse_handler.drag_start_y),5), -5)
   mouse_handler.yaw_rate_ref = max(min(.01*(x-mouse_handler.drag_start_x),1.5), -1.5)

def mouseFunc(button,state,x,y):
   global mouse_handler
   global refType
   global position
   global zCmd
   if (state == GLUT_DOWN):
      mouse_handler.drag_start_x = x
      mouse_handler.drag_start_y = y
   else:
      mouse_handler.vx_ref = 0
      mouse_handler.yaw_rate_ref = 0

if __name__ == '__main__':
    setup()
    main()

