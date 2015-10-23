import numpy as np
import scipy as sc
import math
class Propeller:


# Static Parameters
    rho = 1.225
    J = []       # moment of inertia in kgm^2
    mass = []    # mass in kg
    radius = []  # radius in m  
    cT = []      # zero-velocity thrust coefficient
    cQ = []
    cM = []
# Methods

    # constructor
    # radius = .127 : 5 in radius
    # radius = .1905 : 7.5 in radius
    def __init__(self, J = 0.00026, mass = .01255, radius = 0.127, cT = 0.02, cQ = .003, cM = 0.0001): # these are defaults, based on 10.98 DJI prop tests
        self.J = J
        self.mass = mass
        self.radius = radius
        self.cT = cT
        self.cQ = cQ
        self.cM = cM
        return

    def getMass(self):
       return self.mass

    # Thrust coefficient correction as function of inflow
    def cTo(self, vPar):
        # calculates baseline thrust coefficient
        # eventually based on lookup table from data
        if(vPar < 0):
            coef = self.cT
        else:
            coef = max(0,self.cT  - self.cT*(-0.03*math.sqrt(vPar)))
        return coef

    # Thrust coefficient correction as function of crossflow
    def cTh(self, vPerp):
        coef = 1 + .1*np.log(vPerp+1)
        return coef        

    # Torque coefficient correction as function of inflow
    def cQo(self, vPar):
        # calculates baseline thrust coefficient
        # eventually based on lookup table from data
        if(vPar < 0):
            coef = self.cQ
        else:
            coef = self.cQ - self.cQ*(-0.03*math.sqrt(vPar))
        return coef

    # Thrust coefficient correction as function of crossflow
    def cQh(self, vPerp):
        coef = 1
        return coef

    # Calculate the force (from thrust) of the prop on the vehicle, as a function of wind and prop speeds 
    def calculateThrust(self, omega, apparentWind=np.zeros((3,1))):
        # first break wind into components
        vParallel = apparentWind[2]
        vPerpendicular = math.sqrt(apparentWind[0]**2 + apparentWind[1]**2)
        Thrust = self.cTo(vParallel)*self.cTh(vPerpendicular)*0.5*self.rho*math.pi*(self.radius**4)*omega**2
        return Thrust

    # Calculate axial aero torque
    def calcAxialAeroTorque(self,omega,apparentWind=np.zeros((3,1))):
        # first break wind into components
        vParallel = apparentWind[2]
        vPerpendicular = math.sqrt(apparentWind[0]**2 + apparentWind[1]**2)
        Torque= self.cQo(vParallel)*self.cQh(vPerpendicular)*0.5*self.rho*math.pi*(self.radius**5)*omega**2
        return Torque

    def calcTransverseTorque(self,omega,apparentWind = np.zeros((3,1))):
        # prop will produce a "pitch up" moment as a function of the crossflow
        # this returns a vector
        vPerp = math.sqrt(apparentWind[0]**2 + apparentWind[1]**2)
        normWind = np.sqrt(np.dot(apparentWind.T,apparentWind))
	if(normWind == 0):
            return apparentWind # zeros
        else:
            eWind = apparentWind/normWind
        momentDir = np.cross(eWind.T,np.array([0,0,1])).T # wind X z
        momentMag = self.cMo(vPerp)*0.5*self.rho*math.pi*(self.radius**5)*omega**2
        return momentMag*momentDir

    def cMo(self,vPerp):
        return self.cM 

    # Wrapper to calculate all forces and moments
    def calculateForcesAndMoments(self, omega, apparentWind, direction):
        # windvelocity in body-xyz coordinates
        # omega is angular velocity of motor in rad/sec
        # direction is +1 for CW and -1 for CCW
        # for now this just does force due to thrust, but could do drag too
        self.cT = np.interp(omega,omegaLookup,cTlookup)
        F = self.calculateThrust(omega,apparentWind)
        ForceVector = np.array([[0.,0.,-F]]).T
        M = self.calcAxialAeroTorque(omega,apparentWind)
        MomentVector = np.array([[0,0,-direction*M]]).T + self.calcTransverseTorque(omega,apparentWind)
        #print MomentVector
        return [ForceVector, MomentVector] #[[F], [M]]


omegaLookup = [0,
0,
0,
106.814150222053,
131.946891450771,
160.221225333079,
182.212373908208,
201.061929829747,
219.911485751286,
238.761041672824,
254.469004940773,
270.176968208722,
285.884931476671,
295.309709437441,
311.01767270539,
323.584043319749,
339.292006587698,
351.858377202057,
364.424747816416,
376.991118430775,
389.557489045134,
398.982267005904,
408.407044966673,
420.973415581032,
430.398193541802,
439.822971502571,
446.106156809751,
455.53093477052,
464.955712731289,
474.380490692059,
480.663675999238,
490.088453960008,
499.513231920777,
505.796417227957,
512.079602535136,
521.504380495906,
530.929158456675,
540.353936417444,
549.778714378214,
556.061899685393,
565.486677646163,
574.911455606932,
584.336233567702,
593.761011528471,
606.32738214283,
606.32738214283,
622.035345410779,
631.460123371548,
640.884901332318,
650.309679293087,
659.734457253857,
666.017642561036,
678.584013175395,
688.008791136165,
697.433569096934,
706.858347057704,
716.283125018473,
719.424717672063,
728.849495632832,
738.274273593601,
750.840644207961,
763.40701482232	,
763.40701482232	]

# lookup table for DJI 10.38 prop
cTlookup = [0	,
0	,
0	,
0.001169706353063	,
0.001552060570012	,
0.00244524924916	,
0.002753857456734	,
0.006105539086975	,
0.008678925035976	,
0.010113641349911	,
0.011830848639696	,
0.013214381831516	,
0.014450587660778	,
0.015532229502853	,
0.015732198860124	,
0.016430485489777	,
0.01669728499703	,
0.017269986021894	,
0.017690913071321	,
0.018157443823866	,
0.018384761360048	,
0.019005910379148	,
0.019123083288312	,
0.019187314651495	,
0.019306144272855	,
0.019509311505148	,
0.01996363465144	,
0.02013264865854	,
0.020239566139109	,
0.020380688154864	,
0.020539470325957	,
0.020622891514969	,
0.020647658377	,
0.020694877597086	,
0.020702987461722	,
0.020705062628629	,
0.020828249003774	,
0.020902416215955	,
0.021229562053193	,
0.021234418430245	,
0.021325416821456	,
0.021379910433074	,
0.021397363221787	,
0.02140898838783	,
0.021450389623438	,
0.021545171941271	,
0.021581862917956	,
0.02172444344397	,
0.021763230968437	,
0.021797356011814	,
0.021815479683592	,
0.021881777384143	,
0.021899094235382	,
0.021942324521223	,
0.022030403089831	,
0.022137079210496	,
0.022150756701749	,
0.022170342887992	,
0.022227567697283	,
0.022277181450312	,
0.022285299253218	,
0.022312303774462	,
0.022410418939143]	
