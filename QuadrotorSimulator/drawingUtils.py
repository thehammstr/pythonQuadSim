import Multirotor
import numpy as np
# openGL imports
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

def drawQuad(att = [0,0,0], pos = np.zeros((3,1)), color = [1.,0.,0.,1.],quad = []):
    glPushMatrix()
    xLen = 1
    yLen = 1
    zLen = 1
    glTranslate(pos[0,0],pos[1,0],pos[2,0])
    glPushMatrix()
    glRotate(att[2],0,0,1)
    glRotate(att[1],0,1,0)
    glRotate(att[0],1,0,0)
    # rotor blades
    drawQuad.Theta1 += quad.motorList[0].motorState[1]*1
    drawQuad.Theta2 += quad.motorList[1].motorState[1]*1
    drawQuad.Theta3 += quad.motorList[2].motorState[1]*1
    drawQuad.Theta4 += quad.motorList[3].motorState[1]*1
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
    drawAxes()
    glPushMatrix()
    # Rotate before Translate (otherwise weird things happen)
    glRotate(45., 0.0, 0.0, 1.0)
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color)
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emissionColor)
    glutSolidCube(.5)
    glPopMatrix()
    glPopMatrix()
    glPopMatrix()
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
    glDisable(GL_CULL_FACE)
    q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(coneRadius,2*coneRadius,20,20)
    glPopMatrix()

    # y-direction
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0,1,0,1]);
    glRotate(90,-1,0,0)
    # draw base

    glDisable(GL_CULL_FACE)
    #q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(coneRadius,2*coneRadius,20,20)
    glPopMatrix()

    # z-direction
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [1,1,0,1]);
    # draw base

    glDisable(GL_CULL_FACE)
    #q = gluNewQuadric()
    gluCylinder(q,coneRadius/2,coneRadius/2,arrowLength,32,32);
    glTranslate(0,0,arrowLength)
    glutSolidCone(coneRadius,2*coneRadius,20,20)
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
    step = 2
    for ii in range(-gridSize,gridSize+step,step):
       glVertex3i(gridSize,ii,0)
       glVertex3i(-gridSize,ii,0)
       glVertex3i(ii,gridSize,0)
       glVertex3i(ii,-gridSize,0)

    glEnd()

