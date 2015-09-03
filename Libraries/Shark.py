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

def drawFin(length = 1, height = 1, sweep = 1,thickness = .1):
    angle = math.tan(sweep)
    glBegin(GL_TRIANGLES)
    
    glVertex3f(length/2,thickness,0.0)
    glVertex3f(-height*angle,0.,-height)
    glVertex3f(-length/2,thickness,0.0)
    
    glVertex3f(length/2,-thickness,0.0)
    glVertex3f(-height*angle,0.,-height)
    glVertex3f(-length/2,-thickness,0.0)

    glVertex3f(-length/2,-thickness,0.0)
    glVertex3f(-height*angle,0.,-height)
    glVertex3f(-length/2,thickness,0.0)

    glVertex3f(length/2,-thickness,0.0)
    glVertex3f(-height*angle,0.,-height)
    glVertex3f(length/2,thickness,0.0)
    
    glEnd()

def drawShark(att = [0,0,0], pos = np.zeros((3,1)), tailAngle = 0):
    color = [.6,.6,.6,1.]
    emissionColor = [.6,.6,.6,1.]
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    # body
    print tailAngle
    lgt = 6
    ht = 1
    glPushMatrix()
    glRotate(90,0,1,0)
    glutSolidCone(.5,lgt/4,20,20)
    glPopMatrix()
    glPushMatrix()
    glRotate(-90,0,1,0)
    glutSolidCone(.5,3*lgt/4,20,20)
    glPopMatrix()
    # tail
    glPushMatrix()
    tailW = 1.2
    glTranslatef(-lgt + tailW*2,0,0.)
    glRotate(180./np.pi*tailAngle,0,0,1)
    drawFin(1.2,1.5,1.1,.1)
    drawFin(1.2,-1.1,-.8,.1)
    glPopMatrix()

    # fins
    # dorsal
    glPushMatrix()
    glTranslatef(-1.2,0,0.)
    drawFin(1.2,1.2,.8,.2)
    glPopMatrix()
    # pectoral
    glPushMatrix()
    glRotate(100,1,0,0)
    drawFin(1,1.5,.8,.2)
    glPopMatrix()
    glPushMatrix()
    glRotate(-100,1,0,0)
    drawFin(1,1.5,.8,.2)
    glPopMatrix()
    '''
    global Quad
    glPushMatrix() 
    xLen = 1
    yLen = 1
    zLen = 1
    glTranslate(pos[0,0],pos[1,0],pos[2,0])
    glPushMatrix()
    glRotate(att[2],0,0,1)
    glRotate(att[1],0,1,0)
    glRotate(att[0],1,0,0)
    # drawBody
    # Rotate before Translate (otherwise weird things happen)
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidCone(.2,.7,20,20)
    glPopMatrix()

    
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
    glPopMatrix()
    '''


