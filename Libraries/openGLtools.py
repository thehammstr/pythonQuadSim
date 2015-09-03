# openGL imports
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from math import floor

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


def drawAnchor(position, step = 1):
    glBegin(GL_LINES)
    glVertex3f(position[0,0],position[1,0],position[2,0])
    glVertex3f(position[0,0],position[1,0],0)
    glEnd()
    for ii in range(0,int(floor(position[2,0])),step):
        glPushMatrix()
        glTranslate(position[0,0],position[1,0],ii)
        glutSolidSphere(.08,20,20)
        glPopMatrix()


def drawHorizGridPlane(level = 0,gridExtent = 1000, cellSize = 5 ):
    
    # ground grid
    glBegin(GL_LINES)
    gridSize = gridExtent
    step = cellSize
    for ii in range(-gridSize,gridSize+step,step):
       glVertex3i(gridSize,ii,level)
       glVertex3i(-gridSize,ii,level)
       glVertex3i(ii,gridSize,level)
       glVertex3i(ii,-gridSize,level)
    
    glEnd()

