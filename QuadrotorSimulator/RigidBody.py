import numpy as np
import scipy as sc
from scipy import integrate
import math

class RigidBody:

# Members
    mass = []
    inertia = np.eye(3)
    position = np.zeros((3,1))
    velocity = np.zeros((3,1))
    angularVelocity = np.zeros((3,1))
    b_R_i = np.eye(3)
