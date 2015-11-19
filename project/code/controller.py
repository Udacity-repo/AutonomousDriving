import numpy as np
import scipy.integrate as integrate

class ControllerObj(object):

    def __init__(self, sensor, u_max=4):
        self.Sensor = sensor
        self.numRays = self.Sensor.numRays
        self.actionSet = np.array([-u_max,0,u_max])

    def computeControlInput(self, state, t, frame, raycastDistance=None):
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

        return u, actionIdx


        # rays = self.Sensor.rays

        # c_1 = 1
        # c_2 = 1
        # c_3 = 1

        # F = distances*0.0

        # for i in range(0,numRays):
        #     r = distances[i]
        #     if r is not None:
        #         r_hat = rays[:,i]
        #         x_hat = np.array(frame.transform.TransformNormal(rays[:,numRays/2]))

        #         G_hat = x_hat

        #         F[i] = np.dot(-c_1 * x_hat, r_hat * (1.0 - (r/c_3))**2.0 / (1.0 + (r/c_3)**3.0))

        # Ftot = np.sum(F)
        # F_G = np.dot(-c_2 * G_hat, x_hat)

        # u = F_G + Ftot
        # print u


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
        firstHalf = np.array((self.distances[0:self.numRays/2]))
        secondHalf = np.array((self.distances[self.numRays/2:]))
        tol = 1e-3;

        inverseFirstHalf = (1/firstHalf)**2
        inverseSecondHalf = (1/secondHalf)**2

        numLeft = np.sum(inverseFirstHalf)
        numRight = np.sum(inverseSecondHalf)

        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0

        u = self.actionSet[actionIdx]
        return u, actionIdx
