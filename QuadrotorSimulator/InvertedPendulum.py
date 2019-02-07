import numpy as np
import math
import AeroQuaternion as AQ
from drawingUtils import drawAxes, drawVectorX
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from math import sin, cos
np.random.seed()


class InvertedPendulum:

# Members
    # Static parameters
    mass = 1.0
    Inertia = np.array([[.023385, 0, 0],[0, .18751, 0],[0, 0, .030992]])
    wheel_radius = .05
    height = .3
    depth = .1
    wheelbase = .30
    cg = .2
    # State variables
    # state = [x \
    #          y  >-> Position of axle center expressed in inertial frame
    #          vx >-> velocity of vehicle axle in inertial, expressed in yawed frame
    #          pitch
    #          yaw
    #          pitch rate q
    #          yaw rate r
    stateVector = np.zeros((1,7))
    reaction_forces = np.zeros((1,2))
    # code overhead
    lastTime = -.01
    v_err_last = 0
    p_err_last = 0
    p_err_integral = 0

# Methods

    # constructor
    def __init__(self,
                 Mass = 1.0,
                 Inertia = np.array([[.023385, 0, 0],[0, .18751, 0],[0, 0, .030992]]),
                 ):
        print("Init called")
        self.mass = Mass
        self.Inertia = Inertia

    def updateState(self,dT,cmd):
        # Initialize force and moment vectors
        pos = self.position()
        vel = self.velocity_n()
        vx = self.vx()
        phi, theta, psi = self.attitude_rad()
        # [vx' phi'' psi''].T
        torque_cmd_l_r = self.simpleController(cmd[0], cmd[1], dT)
        derivs = self.vehicleEOM(torque_cmd_l_r)
        self.set_position(pos + vel * dT)
        self.set_vx(vx + derivs[0,0] * dT)
        self.set_heading_rate(self.heading_rate() + derivs[2, 0] * dT)
        theta_sat = max(min(theta + self.pitch_rate() * dT, np.pi  /2 ), -np.pi / 2)
        self.set_attitude_rad(0, theta_sat, psi + self.heading_rate() * dT)
        self.set_heading_rate(self.heading_rate() + derivs[2, 0] * dT)
        self.set_pitch_rate(self.pitch_rate() + derivs[1, 0] * dT)
        return [self.stateVector,np.zeros((3,1))]

    def simpleController(self, vx_ref, psi_dot_ref, dT):
        """Successive loop PD controller
            Inputs:
                self: (like the "this" pointer in C++ but in python you have to pass it in)
                    Note: self.v_err_last and self.p_err_last should be initialized to 0.
                vx_ref: velocity command in m/s
                psi_dot_ref: heading rate command in rad/s
                dT: timestep of control (for calculating derivative term.)
            Output:
                [Left wheel torque command, right wheel torque command] in Newton meters
        """
        _, pitch, yaw = self.attitude_rad()
        # If we're in an unrecoverable attitude, don't become a snow plow.
        if abs(pitch) > 1:
            # We've fallen. Cut the motors.
            return [0, 0]
        # Successive loop closure for longitudinal control
        # Outer loop
        Kp_o = -.15
        Kd_o = -.025
        v_err = vx_ref - self.vx()
        pitch_sat = np.pi/180*20
        # Saturated PD
        pitch_ref = min(max(Kp_o*(v_err) + Kd_o * (v_err - self.v_err_last)/dT,-pitch_sat),pitch_sat)
        self.v_err_last = v_err
        # Inner loop
        Kp_i = 0.8
        Kd_i = .1
        p_err = pitch_ref - pitch
        Tlong = Kp_i * p_err + Kd_i * (p_err - self.p_err_last)/dT
        self.p_err_last = p_err
        # Lateral control
        heading_rate_err = psi_dot_ref - self.heading_rate()
        Kp_lat = 4.5
        Tlat = Kp_lat * heading_rate_err
        # Mix lateral and longitudinal control
        return [Tlong + Tlat, Tlong - Tlat]


    def vehicleEOM(self,y):
        phi, theta, psi = self.attitude_rad()
        r = self.wheel_radius
        Ixx = self.Inertia[0,0]
        Iyy = self.Inertia[1,1]
        Izz = self.Inertia[2,2]
        Mw = self.mass * 0.1
        g = 9.81
        h = self.height
        m = self.mass
        Tc = y[0]
        Td = y[1]
        vx = self.vx()
        q1d = self.pitch_rate()
        q2d = self.heading_rate()
        l = self.wheelbase/2.
        #--------------------------------
        #
        # BEGIN SYMPY CODE
        #
        #--------------------------------
        x0  =  psi
        x1  =  r*sin(x0)**2
        x2  =  r*cos(x0)**2
        x3  =  x1 + x2
        x4  =  theta
        x5  =  cos(x4)
        x6  =  h*m
        x7  =  x5*x6
        x8  =  x3*x7
        x9  =  l*x8
        x10  =  h**2*m
        x11  =  Iyy + x10 + x8
        x12  =  l*x11
        x13  =  -x12 + x9
        x14  =  l**2
        x15  =  2*Mw
        x16  =  Mw*r**2
        x17  =  0.5*x16
        x18  =  sin(x4)
        x19  =  x18**2
        x20  =  x5**2
        x21  =  Ixx*x19 + Izz*x20 + x10*x19 + x14*x15 + x17
        x22  =  1/x3
        x23  =  1.0*x16*x22
        x24  =  x14*x23
        x25  =  x21*(-x1 - x2)
        x26  =  x17*x22
        x27  =  x21*x26
        x28  =  m + x15
        x29  =  x12*x28
        x30  =  x28*x3
        x31  =  -x30
        x32  =  -x23 + x31 - x7
        x33  =  l*x32*x7
        x34  =  l*x26
        x35  =  x34*(x29 + x33)
        x36  =  -x11*x27 + x35
        x37  =  x21*x3
        x38  =  x24 + x37
        x39  =  x21*(x11*(-x26 + x31) - x32*x8) - x35
        x40  =  x38*x39
        x41  =  x36*(-x24 + x25) - x40
        x42  =  x21*x41
        x43  =  x12*x26*x42
        x44  =  x13*x34
        x45  =  x36*(x21*(x11*x3 - x3**2*x7) - x44) - x39*x44
        x46  =  x21*x38
        x47  =  x45*x46
        x48  =  1/x11
        x49  =  1/x21
        x50  =  q2d
        x51  =  x50**2
        x52  =  x18*x51
        x53  =  Tc*x22
        x54  =  Td*x22
        x55  =  q1d
        x56  =  -m*x18*(h*x19*x51 + h*x55**2) - x20*x52*x6 + x53 + x54
        x57  =  1/x41
        x58  =  1/x36
        x59  =  x57*x58
        x60  =  x56*x59
        x61  =  x49*x60
        x62  =  x48*x61
        x63  =  x23 + x30 + x7
        x64  =  x18*x6
        x65  =  x5*x52
        x66  =  Izz*x5
        x67  =  Ixx*x65 + Tc + Td + g*x64 + x10*x65 - x52*x66
        x68  =  x59*x67
        x69  =  x49*x68
        x70  =  x48*x69
        x71  =  -x29 - x33
        x72  =  l*x36
        x73  =  -x38*x71 - 2*x72
        x74  =  x50*x55
        x75  =  x5*x74
        x76  =  x18*x74
        x77  =  x57*(l*x53 - l*x54 - 2*x10*x18*x75 + x18*(-Ixx*x75 - Iyy*x75 + x66*x74) - x5*(Ixx*x76 - Iyy*x76 - Izz*x76) + x50*x64*vx)
        x78  =  x58*x77
        x79  =  x49*x78
        x80  =  x48*x79
        x81  =  x11*x42
        x82  =  x14*x7
        x83  =  x26*x82
        x84  =  x36*(x37*x7 + x83) + x39*x83
        x85  =  x46*x84
        x86  =  x26*x72 + x34*x39
        x87  =  x46*x86
        x88  =  x42*x63
        x89  =  x21*x40
        x90  =  x46*x57
        # Dynamic equations:
        u1_dot = x62*(x11*x47 - x13*x43) + x70*(x41*(-x21*x44*x63 + x25*x36) + x47*x63) + x80*(x41*(x36*(x12 - x9) - x44*x71) - x45*x73)
        u2_dot = x62*(x11*x85 + x81*x83) + x70*(x41*(x21*x36 + x27*x63*x82) + x63*x85) + x80*(x41*(x7*x72 + x71*x83) - x73*x84)
        u3_dot = x61*(x11*x87 + x43) + x69*(x34*x88 + x63*x87) + x79*(x41*(x34*x71 + x36) - x73*x86)
        # Reaction forces
        u2 = q1d
        u3 = self.heading_rate()
        omega_l = (-1 / r)*(vx + q2d*l)
        omega_r = (-1 / r)*(vx - q2d*l)
        A_reaction = np.array([[-1, -1],[l,-l]])
        b_reaction = np.array([[g*m - m*(h*u2**2 + h*u3**2*sin(theta)**2)*cos(theta) + m*(h*u3**2*sin(theta)*cos(theta) - h*u2_dot)*sin(theta)],
            [0.5*Mw*r**2*(omega_l + u2)*u3 + 0.5*Mw*r**2*(omega_r + u2)*u3 - h*m*(h*(-u2*u3*cos(theta) - sin(theta)*u3_dot) - h*u2*u3*cos(theta))*cos(theta) - h*m*u3*vx*cos(theta) - (0.25*Mw*r**2*(-omega_l - u2) + 0.25*Mw*r**2*(omega_l + u2))*u3 - (0.25*Mw*r**2*(-omega_r - u2) + 0.25*Mw*r**2*(omega_r + u2))*u3 - (Ixx*(-u2*u3*cos(theta) - sin(theta)*u3_dot) - Iyy*u2*u3*cos(theta) + Izz*u2*u3*cos(theta))*cos(theta) - (Ixx*u2*u3*sin(theta) - Iyy*u2*u3*sin(theta) + Izz*(-u2*u3*sin(theta) + cos(theta)*u3_dot))*sin(theta)]])
        self.reaction_forces = np.linalg.solve(A_reaction, b_reaction).T
        print(self.reaction_forces[0], sum(self.reaction_forces[0]))
        #--------------------------------
        #
        # END SYMPY CODE
        #
        #--------------------------------
        return np.array([[u1_dot, u2_dot, u3_dot]]).T

    # Return roll, pitch, yaw in radians.
    def attitude_rad(self):
        return np.hstack((np.array([0]), self.stateVector[0,3:5]))
    def set_attitude_rad(self, phi, theta, psi):
        self.stateVector[0,3] = theta
        self.stateVector[0,4] = psi
    # Return xyz
    def position(self):
        return np.array([np.hstack((self.stateVector[0,0:2], np.array([-self.wheel_radius])))]).T
    def set_position(self, pos_vec):
        self.stateVector[0,0] = pos_vec[0,0]
        self.stateVector[0,1] = pos_vec[1,0]
    # Return velocity in Nxyz
    def velocity_n(self):
        Vbody = np.array([[self.stateVector[0,2], 0, 0]]).T
        _, _,psi = self.attitude_rad()
        v_Q_i = AQ.Quaternion([0, 0, 180.0/np.pi*psi])
        return np.dot(v_Q_i.asRotMat.T , Vbody)
    def vx(self):
        return self.stateVector[0,2]
    def set_vx(self, vx):
        self.stateVector[0,2] = vx
    def heading_rate(self):
        return self.stateVector[0,6]
    def set_heading_rate(self, rate):
        self.stateVector[0, 6] = rate
    def pitch_rate(self):
        return self.stateVector[0,5]
    def set_pitch_rate(self,rate):
        self.stateVector[0, 5] = rate


    def draw(self,color = [1,0,0,1],wheel_color = [0,1,0,1]):
        glPushMatrix()
        glTranslate(self.stateVector[0,0],self.stateVector[0,1], -self.wheel_radius)
        glPushMatrix()
        r_rad, p_rad, y_rad = self.attitude_rad()
        glRotate(180.0/np.pi*y_rad,0,0,1)
        glRotate(180.0/np.pi*p_rad,0,1,0)
        glRotate(180.0/np.pi*r_rad,1,0,0)
        # Body
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, color)
        q = gluNewQuadric()
        glPushMatrix()
        glRotate(90.,1.0,0.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.wheelbase/2.,32,32);
        glPopMatrix()
        glPushMatrix()
        glRotate(-90.,1.0,0.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.wheelbase/2.,32,32);
        glPopMatrix()
        glPushMatrix()
        glRotate(180.,0.0,1.0,0.0)
        gluCylinder(q,self.wheel_radius/4.,self.wheel_radius/4.,self.height,32,32);
        glPopMatrix()
        glPushMatrix()
        glTranslate(0,0,-self.height)
        glRotate(180.,0.0,1.0,0.0)
        gluCylinder(q,self.depth,self.depth,self.depth,32,32);
        # Forward Reference
        glPushMatrix()
        glTranslate(0,0,self.depth/2)
        glRotate(180.,0.0,1.0,0.0)
        drawVectorX(.05, .2)
        glPopMatrix()
        glPopMatrix()
        # Left wheel
        glPushMatrix()
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, wheel_color)
        glTranslate(0.0,-self.wheelbase/2,0.0)
        glPushMatrix()
        glRotate(90.0,1.0,0.0,0.0)
        glutSolidTorus(0.01,self.wheel_radius,20,20)
        glPopMatrix()
        glPopMatrix()
        # Right wheel
        glPushMatrix()
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, wheel_color)
        glTranslate(0.0,self.wheelbase/2,0.0)
        glRotate(90.0,1.0,0.0,0.0)
        glutSolidTorus(0.01,self.wheel_radius,20,20)
        glPopMatrix()

        glPopMatrix() # body rotation
        glPopMatrix() # body translation

