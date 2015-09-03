#! /usr/bin/python
import AeroQuaternion as AQ
import numpy as np




Q1 = AQ.Quaternion(np.array([0,0,0]))
Q2 = AQ.Quaternion(np.array([0,0,90]))
Q3 = AQ.Quaternion(np.array([0,90,0]))
Q4 = AQ.Quaternion(np.array([0,0,180]))
Q5 = AQ.Quaternion(np.array([90,0,0]))
Q6 = AQ.Quaternion(np.array([0,45,0]))
Q7 = AQ.Quaternion(np.array([45,0,0]))



v = np.array([[0,1,0]])

print 'printing rotations of ',v.T

print '0 : ', AQ.rotateVector(Q1,v).T
print '90 yaw: ', AQ.rotateVector(Q2,v).T
print 'with matrix: ', np.dot(Q2.asRotMat,v.T).T
print '90 pitch: ', AQ.rotateVector(Q3,v).T
print 'with matrix: ', np.dot(Q3.asRotMat,v.T).T
print '90 roll: ', AQ.rotateVector(Q5,v).T, 
print 'with matrix: ', np.dot(Q5.asRotMat,v.T).T
print '45 roll: ', AQ.rotateVector(Q7,v).T
print 'with matrix: ', np.dot(Q7.asRotMat,v.T).T
print '45 pitch: ', AQ.rotateVector(Q6,v).T
print 'with matrix: ', np.dot(Q6.asRotMat,v.T).T
print '180 yaw: ', AQ.rotateVector(Q4,v).T
print 'with matrix: ', np.dot(Q4.asRotMat,v.T).T


print 'Q7 as euler',Q7.asEuler


