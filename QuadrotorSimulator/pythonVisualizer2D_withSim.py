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
# shark imports
import Shark

############################
# 
#    Begin setup
#
############################
name = 'Quadrotor Visualizer'
zoomLevel = 100
position = np.array([[0,0,0]]).T
heading = 0
polygons = [np.array([[10,10],[20,10],[20,20]]), -np.array([[10,10],[20,10],[20,20],[10,20]])]
path = [np.array([[0.,4.,0.]]) , np.array([[1.,100.,0.]]) , np.array([[100,0.,0.]]) ]
#path = [np.array([[0,10,0]]) , np.array([[0,13,0]]) ]
dragStart_x = 0
dragStart_y = 0


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
    step = 5
    for ii in range(-gridSize,gridSize+step,step):
       glVertex3i(gridSize,ii,1)
       glVertex3i(-gridSize,ii,1)
       glVertex3i(ii,gridSize,1)
       glVertex3i(ii,-gridSize,1)
    
    glEnd()

def drawPolygons(polygons):
  for poly in polygons:
    glBegin(GL_POLYGON)
    for i in range(poly.shape[0]):
      glVertex3f(poly[i,0],poly[i,1],0)
    glEnd()


def drawPlannedPath(path):
  print len(path)
  if (len(path) >= 2):
    for i in range(len(path)-1):
      glBegin(GL_LINES)
      print path[i][0,0] , path[i][0,1]
      print path[i+1][0,0] , path[i+1][0,1]
      glVertex3f(path[i][0,0],path[i][0,1],0)
      glVertex3f(path[i+1][0,0],path[i+1][0,1],0)
      glEnd()



def getValues():
    global position
    global heading
    global plannedPath
    global polygons

    glutPostRedisplay()


  
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
    glutIdleFunc(getValues)
    glutKeyboardFunc(keyboardHandler)
    glutMotionFunc(dragFunc)
    glutMouseFunc(mouseFunc)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(40.,1.,1.,1000.)
    glMatrixMode(GL_MODELVIEW)

    glutMainLoop()
    return

def display():
    startTime = clock.time()
    #print startTime,
    global position
    global heading
    global zoomLevel
    global polygons
    global path
    glLoadIdentity()
    # display parameters
    gl_R_ned = AQ.Quaternion(np.array([0,180,-90]))
    print position
    posGL = np.dot(gl_R_ned.asRotMat,position)
    gluLookAt(posGL[0],posGL[1], zoomLevel,
              posGL[0],posGL[1],posGL[2],	
              0,1,0)
    glPushMatrix()

    # rotate into aero convention
    glRotate(180.,1.,1.,0)

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glColor4f(0.,0.,1.,.8)
    color = [0,0.,.5,.1]
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color)
    # draw ground
    drawEnvironment()
    color = [1.,0.,0.,1.]
    emissionColor = [.5,0.,0.,.1]
    drawAxes()
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)

    # push quadrotor position on stack
    glPushMatrix() 
    glTranslate(position[0,0],position[1,0],0)
    glRotate(heading,0,0,1)
    # draw quadrotor
    drawQuad(color = [1.,0.,0.,1.])

    # draw body axes
    drawAxes()
    glPopMatrix()

    drawPolygons(polygons)

    # change color
    color = [0.0,1.,0.,1.]
    glMaterialfv(GL_FRONT,GL_EMISSION,color)

    drawPlannedPath(path)

    glPopMatrix()
    glutSwapBuffers()
    #print clock.time() - startTime

    return


def keyboardHandler(key,x,y):
    global heading
    global zoomLevel
    if (key == '1'):
       heading -= 1
    if (key == '2'):
       heading += 1
    if (key == "w"):
       zoomLevel = max(zoomLevel-5,10)
    elif (key == "s"):
       zoomLevel += 5


def dragFunc(x,y):
   global position
   global dragStart_x
   global dragStart_y
   global yawCmd
   #print 'x: ', x, 'y: ',y
   if (np.abs(x-dragStart_x) > 20):
      position[1,0] += 1*np.sign(x-dragStart_x)
   if (np.abs(y-dragStart_y) > 20):
      position[0,0] -= 1*np.sign(y-dragStart_y)
   print position

def mouseFunc(button,state,x,y):
   global dragStart_x
   global dragStart_y
   global refType
   global reference 
   global position
   if (state == GLUT_DOWN):
      dragStart_x = x
      dragStart_y = y

if __name__ == '__main__': main()

