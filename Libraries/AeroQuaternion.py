import numpy as np
from math import cos, sin, radians, degrees, atan2, asin, acos, sqrt

class Quaternion(object):
   """   
   Example usage::

    >>> from Quaternion import Quat
    >>> quat = Quat((12,45,45))
    >>> quat.ra, quat.dec, quat.roll
    (12, 45, 45)
    >>> quat.q
    array([ 0.38857298, -0.3146602 ,  0.23486498,  0.8335697 ])
    >>> q2 = Quat([ 0.38857298, -0.3146602 ,  0.23486498,  0.8335697])
    >>> q2.ra
    11.999999315925008


   Multiplication and division operators are overloaded for the class to 
   perform appropriate quaternion multiplication and division.

   Example usage::
   
    >>> q1 = Quat((20,30,40))
    >>> q2 = Quat((30,40,50))
    >>> q = q1 / q2

   Performs the operation as q1 * inverse q2

   Example usage::

    >>> q1 = Quat((20,30,40))
    >>> q2 = Quat((30,40,50))
    >>> q = q1 * q2


   :param attitude: initialization attitude for quat

   ``attitude`` may be:
     * another Quat
     * a 4 element array (expects x,y,z,w quat form)
     * a 3 element array (expects ra,dec,roll in degrees)
     * a 3x3 transform/rotation matrix

   """
   def __init__(self, attitude = np.array([0.,0.,0.,1.])):
      self._q = None
      # checks to see if we've been passed a Quat 
      if isinstance(attitude, Quaternion):
         self._set_q(attitude.q)
      else:
         # make it an array and check to see if it is a supported shape
         attitude = np.array(attitude)
         if len(attitude) == 4:
            self._set_q(attitude)
         elif attitude.shape == (3,3):
            self._set_rotationMatrix(attitude)
         elif attitude.shape == (3,):
            self._set_euler(attitude)
         else:
            raise TypeError("attitude is not one of possible types (3 or 4 elements, Quat, or 3x3 matrix)")
   
   def _set_q(self, q):
      """
      Set the value of the 4 element quaternion vector 

      :param q: list or array of normalized quaternion elements
      """
      q = np.array(q,dtype=np.float64)
      #if abs(np.sum(q**2) - 1.0) > 1e-6:
      #   raise ValueError('Quaternion must be normalized so sum(q**2) == 1; use Quaternion.normalize')
      self._q = (q if q[3] > 0 else -q)
      # Erase internal values of other representations

   def _get_q(self):
      """
      Retrieve 4-vector of quaternion elements in [x, y, z, w] form
      
      :rtype: numpy array

      """
      return self._q

   # use property to make this get/set automatic
   q = property(_get_q, _set_q)

   def _set_euler(self, eulerAngles):
      """
         reset quaternion using eulerAngles      
         :param eulerAngles: list or array [ roll, pitch, yaw] in degrees         
      """
      self._q = self._euler2quat(eulerAngles)

   def _get_euler(self):
      """Retrieve euler angles

      :rtype: numpy array
      """
      return self._quat2euler()

   asEuler = property(_get_euler,_set_euler)

   def _set_rotationMatrix(self, R):
      """
      Set the value of the 3x3 rotation/transform matrix body_R_inertial
      
      :param R: 3x3 array/numpy array
      """
      self._q = self.rotationMatrix2quat(np.array(R))

   def _get_rotationMatrix(self):
      """
      Retrieve the value of the 3x3 rotation/transform matrix

      :returns: 3x3 rotation/transform matrix
      :rtype: numpy array
      
      """
      return self._quat2rotationMatrix()

   asRotMat = property(_get_rotationMatrix, _set_rotationMatrix)
##################################################################
#   Conversion functions
##################################################################

   def _quat2rotationMatrix(self):
      """
      Transform a unit quaternion into its corresponding rotation matrix veh_R_inertial      
      :returns: transform matrix
      :rtype: numpy array
      
      """
      x, y, z, w = self.q
      xx2 = 2 * x * x
      yy2 = 2 * y * y
      zz2 = 2 * z * z
      xy2 = 2 * x * y
      wz2 = 2 * w * z
      zx2 = 2 * z * x
      wy2 = 2 * w * y
      yz2 = 2 * y * z
      wx2 = 2 * w * x
      
      rmat = np.empty((3, 3), float)
      rmat[0,0] = 1. - yy2 - zz2
      rmat[0,1] = xy2 - wz2
      rmat[0,2] = zx2 + wy2
      rmat[1,0] = xy2 + wz2
      rmat[1,1] = 1. - xx2 - zz2
      rmat[1,2] = yz2 - wx2
      rmat[2,0] = zx2 - wy2
      rmat[2,1] = yz2 + wx2
      rmat[2,2] = 1. - xx2 - yy2
      
      return rmat.T

   def _euler2quat(self,eulerAngles):
       phi, theta, psi = np.pi/180*eulerAngles
       cphi2 =   cos(phi/2)
       ctheta2 = cos(theta/2)
       cpsi2 =   cos(psi/2)
       sphi2 =   sin(phi/2)
       stheta2 = sin(theta/2)
       spsi2 =   sin(psi/2)

       qw = cphi2*ctheta2*cpsi2 + sphi2*stheta2*spsi2
       qx = sphi2*ctheta2*cpsi2 - cphi2*stheta2*spsi2
       qy = cphi2*stheta2*cpsi2 + sphi2*ctheta2*spsi2
       qz = cphi2*ctheta2*spsi2 - sphi2*stheta2*cpsi2
       return np.array([qx, qy, qz, qw])
       

   def _quat2euler(self):
       qx, qy, qz, w = self._q
       phi = 180./np.pi*atan2(2*(w*qx + qy*qz),1-2*(qx*qx+qy*qy))
       theta = 180./np.pi*asin(2*(w*qy - qz*qx))
       psi = 180./np.pi*atan2(2*(w*qz+qx*qy),1 - 2*(qy*qy+qz*qz))
       return np.array([phi,theta,psi])
##################################################################
#   Operators
##################################################################
   def __div__(self, quat2):
      """
      Divide one quaternion by another.
      
      Example usage::

       >>> q1 = Quat((20,30,40))
       >>> q2 = Quat((30,40,50))
       >>> q = q1 / q2

      Performs the operation as q1 * inverse q2

      :returns: product q1 * inverse q2
      :rtype: Quat

      """
      return self * quat2.inv()


   def __mul__(self, quat2):
      """
      Multiply quaternion by another.

      :returns: product q1 * q2
      :rtype: Quat

      """
      q1 = self.q
      q2 = quat2.q
      vec1 = np.array([q1[0:3]],dtype=np.float64)
      scalar1 = q1[3]
      vec2 = np.array([q2[0:3]],dtype=np.float64)
      scalar2 = q2[3]
      
      vecAnswer = scalar1*vec2 + scalar2*vec1 - np.cross(vec1,vec2)
      scalarAnswer = scalar1*scalar2 - np.dot(vec1,vec2.T)


      mult = np.zeros(4,dtype=np.float64)
      mult[0:3] = vecAnswer[0]
      mult[3] = scalarAnswer
      '''
      mult[0] = q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3]
      mult[1] = q1[2]*q2[0] + q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3]
      mult[2] = -q1[1]*q2[0] + q1[0]*q2[1] + q1[3]*q2[2] + q1[2]*q2[3]
      mult[3] = -q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] + q1[3]*q2[3]
      '''
      return Quaternion(mult)

   def inv(self):
      """
      Invert the quaternion 

      :returns: inverted quaternion
      :rtype: Quat
      """
      
      return Quaternion([-self.q[0], -self.q[1], -self.q[2], self.q[3]])

   def normalize(self):
      self._q = normalize(self.q)


def normalize(array):
   """ 
   Normalize a 4 element array/list/numpy.array for use as a quaternion
   
   :param quat_array: 4 element list/array
   :returns: normalized array
   :rtype: numpy array

   """
   quat = np.array(array)
   return quat / np.sqrt(np.dot(quat, quat))

def slerp(quat1, quat2, alpha):
   """ Spherical linear interpolation between quaternions
       linearly interpolate alpha amout FROM quat1 TO quat2

   """
   qDiff = quat2*quat1.inv()
   w = qDiff.q[3]
   if (w <= .99999):
     the = 2*np.arccos(w)
     x = qDiff.q[0]/np.sqrt(1.-(w*w)) 
     y = qDiff.q[1]/np.sqrt(1.-(w*w))
     z = qDiff.q[2]/np.sqrt(1.-(w*w))
     axis = np.array([[x,y,z]])
   else: 
     the = 0.
     axis = np.array([[1.,1.,1.]])

   if (the > np.pi):
     the -= 2.*np.pi
   if (the < -np.pi):
     the += 2.*np.pi

   qMove = quatFromAxisAngle(axis.T, alpha*the)
   qNew = qMove*quat1
      
   return qNew

def rotateVector(quat,vec):
    # If vec is a vector in inertial coordinates and quat represents a rotation from inertial to body
    # this function will return the vector written in body coordinates
    #print vec
    if (len(vec.shape) == 1):
        vec = np.array([vec]).T
    elif (vec.shape[0] == 1 and vec.shape[1] == 3):
        vec = vec.T
    elif (vec.shape[1] == 1 and vec.shape[0] == 3):
        pass
    else:
        raise TypeError("3x1 or 1x3 vector array required")
    #print np.dot(vec.T,vec)
    if (np.dot(vec.T,vec) <= 1.e-12):
        return vec
    outVec1 = np.dot(quat.asRotMat,vec)
    return outVec1                                 
    ''' print rotatedVector.q
    outVec[0] = rotatedVector.q[0]
    outVec[1] = rotatedVector.q[1]
    outVec[2] = rotatedVector.q[2]'''

def qDotFromOmega(quat,omega):
    # omega is in body-fixed coordinates
    # quat maps inertial to local v' = q.inv*v*q
    output = np.zeros(4)
    q1,q2,q3,q0 = quat.q
    wx,wy,wz = omega
    output[3] = -q1*wx - q2*wy - q3*wz
    output[0] = q0*wx - q3*wy + q2*wz
    output[1] = q3*wx + q0*wy - q1*wz
    output[2] = -q2*wx + q1*wy + q0*wz
    return 0.5*output

def quatFromAxisAngle(axis,angle):
    if (np.abs(angle) > 0.):
       if (np.linalg.norm(axis) != 1.0):
          axis = axis/(np.linalg.norm(axis) + 1e-13)
       x,y,z = axis.T[0]
       q = Quaternion()
       sin2 = np.sin(angle/2.)
       cos2 = np.cos(angle/2.)
       q.q = np.array([x*sin2, y*sin2, z*sin2, cos2])
    else:
       q = Quaternion()
    return q
