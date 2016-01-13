import numpy as np
import scipy.integrate as integrate
import ddapp.objectmodel as om


class ControllerObj(object):

    def __init__(self, sensor, u_max=4, epsilonRand=0.4):
        self.Sensor = sensor
        self.numRays = self.Sensor.numRays
        self.actionSet = np.array([u_max,0,-u_max])
        self.epsilonRand = epsilonRand
        self.actionSetIdx = np.arange(0,np.size(self.actionSet))

    def computeControlInput(self, state, t, frame, raycastDistance=None, randomize=False):
        # test cases
        # u = 0
        # u = np.sin(t)
        if raycastDistance is None:
            self.distances = self.Sensor.raycastAll(frame)
        else:
            self.distances = raycastDistance

        # #Barry 12 controller
        

        #u = self.countStuffController()
        u, actionIdx = self.countInverseDistancesController()

        if randomize:
            if np.random.uniform(0,1,1)[0] < self.epsilonRand:
                # otherActionIdx = np.setdiff1d(self.actionSetIdx, np.array([actionIdx]))
                # randActionIdx = np.random.choice(otherActionIdx)
                actionIdx = np.random.choice(self.actionSetIdx)
                u = self.actionSet[actionIdx]

        return u, actionIdx


    def countStuffController(self):
        firstHalf = self.distances[0:self.numRays/2]
        secondHalf = self.distances[self.numRays/2:]
        tol = 1e-3;

        numLeft = np.size(np.where(firstHalf < self.Sensor.rayLength - tol))
        numRight = np.size(np.where(secondHalf < self.Sensor.rayLength - tol))

        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def countInverseDistancesController(self):
        midpoint = np.floor(self.numRays/2.0)
        leftHalf = np.array((self.distances[0:midpoint]))
        rightHalf = np.array((self.distances[midpoint:]))
        tol = 1e-3;

        inverseLeftHalf = (1.0/leftHalf)**2
        inverseRightHalf = (1.0/rightHalf)**2

        numLeft = np.sum(inverseLeftHalf)
        numRight = np.sum(inverseRightHalf)


        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0


        # print "leftHalf ", leftHalf
        # print "rightHalf", rightHalf
        # print "inverseLeftHalf", inverseLeftHalf
        # print "inverserRightHalf", inverseRightHalf
        # print "numLeft", numLeft
        # print "numRight", numRight

        u = self.actionSet[actionIdx]
        return u, actionIdx


    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame)

class DubinsController(object):

    def __init__(self, sensor):
        self.sensor = sensor
        self.initializeGains()

    def initializeGains(self):
        self.gains = {}
        self.gains['v'] = {'Kp': 1.0, 'Kd': 1.0, 'Ki': 1.0}
        self.gains['psi'] = {'Kp': 1.0, 'Kd': 0.0, 'Ki': 1.0}

    def computeControlInput(self, v, psi, t=None, angleToPath=None, distanceToPath=None):

        v_des = 5.0
        u = np.zeros(2)
        u[0] = -self.gains['v']['Kp']*(v - v_des)

        return u

class DubinsPIDController(object):

    def __init__(self, sensor, car):
        self.car = car
        self.sensor = sensor
        self.initializeGains()
        self.initializeIntegrators()

    def initializeGains(self):
        self.gains = {}
        self.gains['v'] = {'Kp': 1.0, 'Kd': 1.0, 'Ki': 0.0}
        self.gains['crossTrackError'] = {'Kp': 1.0, 'Kd': 0.8, 'Ki': 0.0}
        self.gains['psi'] = {'Kp': 10.0}
    def initializeIntegrators(self):
        self.integrators = {}
        self.integrators['v'] = 0.0
        self.integrators['crossTrackError'] = 0.0

    def resetIntegrators(self):
        self.initializeIntegrators()

    def computeControlInput(self, carState, crossTrackError, crossTrackErrorDeriv, slope, dt,
                            v_des=5.0):
        v_des = 5.0
        u = np.zeros(2)
        v = carState[self.car.idx['v']]
        psi = carState[self.car.idx['psi']]
        u[0] = -self.gains['v']['Kp']*(v - v_des) - self.gains['v']['Ki']*self.integrators['v']
        psi_des = self.gains['crossTrackError']['Kp']*crossTrackError + self.gains['crossTrackError']['Kd']*crossTrackErrorDeriv
        u[1] = self.gains['psi']['Kp']*(psi_des - psi)
        # update the integrators
        self.integrators['v'] += (v-v_des)*dt
        self.integrators['crossTrackError'] += crossTrackError*dt


        # check if we are already on a steep approach, if so, command zero steering angle
        # can also achieve this by putting a cap on crossTrackError that gets multiplied into

        verbose=False
        if verbose:
            print "psi_des", psi_des
            print "psi_actual", psi

        return u


    # assumes we only found 2 walls and wan't to go straight
    def computeCrossTrackError(self, carState, wallsFound, verbose=False):

        # compute average of both
        if len(wallsFound) > 2:
            raise ValueError("this controller only works if we one or two walls")


        if len(wallsFound) == 1:
            # just stay 10m from wall
            raise ValueError("not yet implemented")

        if len(wallsFound) == 2:

            slope = 0.0
            intercept = 0.0
            for wallData in wallsFound:
                slope += wallData['ransacFit'][1]
                intercept += wallData['ransacFit'][0]

            slope = slope/2.0
            intercept = intercept/2.0


        # positive means we are too far the the right, so we need to steer to the left,
        # which corresponds to psi positive
        crossTrackError = intercept
        crossTrackErrorDeriv = carState[self.car.idx['v']]*slope

        if verbose:
            print "crossTrackError", crossTrackError
            print "crossTrackErrorDeriv", crossTrackErrorDeriv

        return crossTrackError, crossTrackErrorDeriv, slope

    def computeControlInputFromWallData(self, carState, wallsFound, dt, v_des=5.0):
        crossTrackError, crossTrackErrorDeriv, slope = self.computeCrossTrackError(carState, wallsFound)
        return self.computeControlInput(carState, crossTrackError, crossTrackErrorDeriv, slope,
                                        dt, v_des=v_des)










